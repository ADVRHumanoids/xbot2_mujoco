#include "xmj_sim_env.h"

void XBotMjSimEnv::mj_control_callback(const mjModel* m, mjData* d){
    instance->xbotmj_control_callback(m, d);
}

void XBotMjSimEnv::xbotmj_control_callback(const mjModel* m, mjData* d){
    if(!xbot2_wrapper)
    {
        return;
    }

    xbot2_wrapper->run(d);
}

XBotMjSimEnv::XBotMjSimEnv(const char* configPath) : xbot2_cfg_path(configPath) {
    
    instance = this; // Set the global instance pointer to this instance

    mjcb_control = XBotMjSimEnv::mj_control_callback; // register global
    // mujoco control callback

    window = glfwCreateWindow(800, 600, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetDropCallback(window, dropCallback);
    glfwSetKeyCallback(window, keyCallback);

    // Initialize MuJoCo and other members
    // ...

    loadModel(); // Load the initial model
}

XBotMjSimEnv::~XBotMjSimEnv() {
    if (simThread.joinable()) {
        settings.exitrequest = 1;
        simThread.join();
    }

    fprintf(stderr, "destroying xbot2 wrapper \n");
    xbot2_wrapper.reset();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

void XBotMjSimEnv::run() {

    init();
    
    running = true;
    
    simThread = std::thread(&XBotMjSimEnv::simulate, this);

    // event loop
    while( !glfwWindowShouldClose(window) && !settings.exitrequest )
    {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if( settings.loadrequest==1 )
            loadModel();
        else if( settings.loadrequest>1 )
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    running = false;
}

void XBotMjSimEnv::init() {
    // print version, check compatibility
    printf("MuJoCo Pro version %.2lf\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");
    // init GLFW, set timer callback (milliseconds)
    if (!glfwInit())
        mju_error("could not initialize GLFW");
    mjcb_time = timer;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_VISIBLE, 1);

    // get videomode and save
    vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());

    window = glfwCreateWindow((2*vmode.width)/3, (2*vmode.height)/3,
                              "Simulate", NULL, NULL);
    if( !window )
    {
        glfwTerminate();
        mju_error("could not create window");
    }

    // save window position and size
    glfwGetWindowPos(window, windowpos, windowpos+1);
    glfwGetWindowSize(window, windowsize, windowsize+1);

    // make context current, set v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(settings.vsync);

    // init abstract visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    profilerinit();
    sensorinit();

    // make empty scene
    mjv_defaultScene(&scn);
    mjv_makeScene(NULL, &scn, maxgeom);

    // select default font
    int fontscale = uiFontScale(window);
    settings.font = fontscale/50 - 1;

    // make empty context
    mjr_defaultContext(&con);
    mjr_makeContext(NULL, &con, fontscale);

    // set GLFW callbacks
    uiSetCallback(window, &uistate, uiEvent, uiLayout);
    glfwSetWindowRefreshCallback(window, render);
    glfwSetDropCallback(window, drop);

    // init state and uis
    memset(&uistate, 0, sizeof(mjuiState));
    memset(&ui0, 0, sizeof(mjUI));
    memset(&ui1, 0, sizeof(mjUI));
    ui0.spacing = mjui_themeSpacing(settings.spacing);
    ui0.color = mjui_themeColor(settings.color);
    ui0.predicate = uiPredicate;
    ui0.rectid = 1;
    ui0.auxid = 0;
    ui1.spacing = mjui_themeSpacing(settings.spacing);
    ui1.color = mjui_themeColor(settings.color);
    ui1.predicate = uiPredicate;
    ui1.rectid = 2;
    ui1.auxid = 1;

    // populate uis with standard sections
    mjui_add(&ui0, defFile);
    mjui_add(&ui0, defOption);
    mjui_add(&ui0, defSimulation);
    mjui_add(&ui0, defWatch);
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
}

void XBotMjSimEnv::profilerinit() {

    int i, n;

    // set figures to default
    mjv_defaultFigure(&figconstraint);
    mjv_defaultFigure(&figcost);
    mjv_defaultFigure(&figtimer);
    mjv_defaultFigure(&figsize);

    // titles
    strcpy(figconstraint.title, "Counts");
    strcpy(figcost.title, "Convergence (log 10)");
    strcpy(figsize.title, "Dimensions");
    strcpy(figtimer.title, "CPU time (msec)");

    // x-labels
    strcpy(figconstraint.xlabel, "Solver iteration");
    strcpy(figcost.xlabel, "Solver iteration");
    strcpy(figsize.xlabel, "Video frame");
    strcpy(figtimer.xlabel, "Video frame");

    // y-tick nubmer formats
    strcpy(figconstraint.yformat, "%.0f");
    strcpy(figcost.yformat, "%.1f");
    strcpy(figsize.yformat, "%.0f");
    strcpy(figtimer.yformat, "%.2f");

    // colors
    figconstraint.figurergba[0] =   0.1f;
    figcost.figurergba[2] =         0.2f;
    figsize.figurergba[0] =         0.1f;
    figtimer.figurergba[2] =        0.2f;
    figconstraint.figurergba[3] =   0.5f;
    figcost.figurergba[3] =         0.5f;
    figsize.figurergba[3] =         0.5f;
    figtimer.figurergba[3] =        0.5f;

    // legends
    strcpy(figconstraint.linename[0], "total");
    strcpy(figconstraint.linename[1], "active");
    strcpy(figconstraint.linename[2], "changed");
    strcpy(figconstraint.linename[3], "evals");
    strcpy(figconstraint.linename[4], "updates");
    strcpy(figcost.linename[0], "improvement");
    strcpy(figcost.linename[1], "gradient");
    strcpy(figcost.linename[2], "lineslope");
    strcpy(figsize.linename[0], "dof");
    strcpy(figsize.linename[1], "body");
    strcpy(figsize.linename[2], "constraint");
    strcpy(figsize.linename[3], "sqrt(nnz)");
    strcpy(figsize.linename[4], "contact");
    strcpy(figsize.linename[5], "iteration");
    strcpy(figtimer.linename[0], "total");
    strcpy(figtimer.linename[1], "collision");
    strcpy(figtimer.linename[2], "prepare");
    strcpy(figtimer.linename[3], "solve");
    strcpy(figtimer.linename[4], "other");

    // grid sizes
    figconstraint.gridsize[0] = 5;
    figconstraint.gridsize[1] = 5;
    figcost.gridsize[0] = 5;
    figcost.gridsize[1] = 5;
    figsize.gridsize[0] = 3;
    figsize.gridsize[1] = 5;
    figtimer.gridsize[0] = 3;
    figtimer.gridsize[1] = 5;

    // minimum ranges
    figconstraint.range[0][0] = 0;
    figconstraint.range[0][1] = 20;
    figconstraint.range[1][0] = 0;
    figconstraint.range[1][1] = 80;
    figcost.range[0][0] = 0;
    figcost.range[0][1] = 20;
    figcost.range[1][0] = -15;
    figcost.range[1][1] = 5;
    figsize.range[0][0] = -200;
    figsize.range[0][1] = 0;
    figsize.range[1][0] = 0;
    figsize.range[1][1] = 100;
    figtimer.range[0][0] = -200;
    figtimer.range[0][1] = 0;
    figtimer.range[1][0] = 0;
    figtimer.range[1][1] = 0.4f;

    // init x axis on history figures (do not show yet)
    for( n=0; n<6; n++ )
        for( i=0; i<mjMAXLINEPNT; i++ )
        {
            figtimer.linedata[n][2*i] = (float)-i;
            figsize.linedata[n][2*i] = (float)-i;
        }
}

void XBotMjSimEnv::profilerupdate() {
    int i, n;

    // update constraint figure
    figconstraint.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for( i=1; i<5; i++ )
        figconstraint.linepnt[i] = figconstraint.linepnt[0];
    if( m->opt.solver==mjSOL_PGS )
    {
        figconstraint.linepnt[3] = 0;
        figconstraint.linepnt[4] = 0;
    }
    if( m->opt.solver==mjSOL_CG )
        figconstraint.linepnt[4] = 0;
    for( i=0; i<figconstraint.linepnt[0]; i++ )
    {
        // x
        figconstraint.linedata[0][2*i] = (float)i;
        figconstraint.linedata[1][2*i] = (float)i;
        figconstraint.linedata[2][2*i] = (float)i;
        figconstraint.linedata[3][2*i] = (float)i;
        figconstraint.linedata[4][2*i] = (float)i;

        // y
        figconstraint.linedata[0][2*i+1] = (float)d->nefc;
        figconstraint.linedata[1][2*i+1] = (float)d->solver[i].nactive;
        figconstraint.linedata[2][2*i+1] = (float)d->solver[i].nchange;
        figconstraint.linedata[3][2*i+1] = (float)d->solver[i].neval;
        figconstraint.linedata[4][2*i+1] = (float)d->solver[i].nupdate;
    }

    // update cost figure
    figcost.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for( i=1; i<3; i++ )
        figcost.linepnt[i] = figcost.linepnt[0];
    if( m->opt.solver==mjSOL_PGS )
    {
        figcost.linepnt[1] = 0;
        figcost.linepnt[2] = 0;
    }

    for( i=0; i<figcost.linepnt[0]; i++ )
    {
        // x
        figcost.linedata[0][2*i] = (float)i;
        figcost.linedata[1][2*i] = (float)i;
        figcost.linedata[2][2*i] = (float)i;

        // y
        figcost.linedata[0][2*i+1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].improvement));
        figcost.linedata[1][2*i+1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].gradient));
        figcost.linedata[2][2*i+1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].lineslope));
    }

    // get timers: total, collision, prepare, solve, other
    mjtNum total = d->timer[mjTIMER_STEP].duration;
    int number = d->timer[mjTIMER_STEP].number;
    if( !number )
    {
        total = d->timer[mjTIMER_FORWARD].duration;
        number = d->timer[mjTIMER_FORWARD].number;
    }
    number = mjMAX(1, number);
    float tdata[5] = {
        (float)(total/number),
        (float)(d->timer[mjTIMER_POS_COLLISION].duration/number),
        (float)(d->timer[mjTIMER_POS_MAKE].duration/number) +
            (float)(d->timer[mjTIMER_POS_PROJECT].duration/number),
        (float)(d->timer[mjTIMER_CONSTRAINT].duration/number),
        0
    };
    tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

    // update figtimer
    int pnt = mjMIN(201, figtimer.linepnt[0]+1);
    for( n=0; n<5; n++ )
    {
        // shift data
        for( i=pnt-1; i>0; i-- )
            figtimer.linedata[n][2*i+1] = figtimer.linedata[n][2*i-1];

        // assign new
        figtimer.linepnt[n] = pnt;
        figtimer.linedata[n][1] = tdata[n];
    }

    // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
    float sdata[6] = {
        (float)m->nv,
        (float)m->nbody,
        (float)d->nefc,
        (float)mju_sqrt((mjtNum)d->solver_nnz),
        (float)d->ncon,
        (float)d->solver_iter
    };

    // update figsize
    pnt = mjMIN(201, figsize.linepnt[0]+1);
    for( n=0; n<6; n++ )
    {
        // shift data
        for( i=pnt-1; i>0; i-- )
            figsize.linedata[n][2*i+1] = figsize.linedata[n][2*i-1];

        // assign new
        figsize.linepnt[n] = pnt;
        figsize.linedata[n][1] = sdata[n];
    }
}

void XBotMjSimEnv::profilershow(mjrRect rect) {
    mjrRect viewport = {
        rect.left + rect.width - rect.width/4,
        rect.bottom,
        rect.width/4,
        rect.height/4
    };
    mjr_figure(viewport, &instance->figtimer, &instance->con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &instance->figsize, &instance->con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &instance->figcost, &instance->con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &instance->figconstraint, &instance->con);
}

void XBotMjSimEnv::sensorinit() {
    // set figure to default
    mjv_defaultFigure(&figsensor);
    figsensor.figurergba[3] = 0.5f;

    // set flags
    figsensor.flg_extend = 1;
    figsensor.flg_barplot = 1;
    figsensor.flg_symmetric = 1;

    // title
    strcpy(figsensor.title, "Sensor data");

    // y-tick nubmer format
    strcpy(figsensor.yformat, "%.0f");

    // grid size
    figsensor.gridsize[0] = 2;
    figsensor.gridsize[1] = 3;

    // minimum range
    figsensor.range[0][0] = 0;
    figsensor.range[0][1] = 0;
    figsensor.range[1][0] = -1;
    figsensor.range[1][1] = 1;
}

void XBotMjSimEnv::sensorupdate() {
    static const int maxline = 10;

    // clear linepnt
    for( int i=0; i<maxline; i++ )
        figsensor.linepnt[i] = 0;

    // start with line 0
    int lineid = 0;

    // loop over sensors
    for( int n=0; n<m->nsensor; n++ )
    {
        // go to next line if type is different
        if( n>0 && m->sensor_type[n]!=m->sensor_type[n-1] )
            lineid = mjMIN(lineid+1, maxline-1);

        // get info about this sensor
        mjtNum cutoff = (m->sensor_cutoff[n]>0 ? m->sensor_cutoff[n] : 1);
        int adr = m->sensor_adr[n];
        int dim = m->sensor_dim[n];

        // data pointer in line
        int p = figsensor.linepnt[lineid];

        // fill in data for this sensor
        for( int i=0; i<dim; i++ )
        {
            // check size
            if( (p+2*i)>=mjMAXLINEPNT/2 )
                break;

            // x
            figsensor.linedata[lineid][2*p+4*i] = (float)(adr+i);
            figsensor.linedata[lineid][2*p+4*i+2] = (float)(adr+i);

            // y
            figsensor.linedata[lineid][2*p+4*i+1] = 0;
            figsensor.linedata[lineid][2*p+4*i+3] = (float)(d->sensordata[adr+i]/cutoff);
        }

        // update linepnt
        figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT-1,
                                          figsensor.linepnt[lineid]+2*dim);
    }
}

void XBotMjSimEnv::sensorshow(mjrRect rect) {
    // constant width with and without profiler
    int width = settings.profiler ? rect.width/3 : rect.width/4;

    // render figure on the right
    mjrRect viewport = {
        rect.left + rect.width - width,
        rect.bottom,
        width,
        rect.height/3
    };
    mjr_figure(viewport, &instance->figsensor, &instance->con);
}

void XBotMjSimEnv::infotext(char* title, char* content, double interval) {
    char tmp[20];

    // compute solver error
    mjtNum solerr = 0;
    if( d->solver_iter )
    {
        int ind = mjMIN(d->solver_iter-1,mjNSOLVER-1);
        solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
        if( solerr==0 )
            solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
    }
    solerr = mju_log10(mju_max(mjMINVAL, solerr));

    // prepare info text
    strcpy(title, "Time\nSize\nCPU\nSolver   \nFPS\nstack\nconbuf\nefcbuf");
    sprintf(content, "%-20.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f",
            d->time,
            d->nefc, d->ncon,
            settings.run ?
                d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) :
                d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
            solerr, d->solver_iter,
            1/interval,
            d->maxuse_stack/(double)d->nstack,
            d->maxuse_con/(double)m->nconmax,
            d->maxuse_efc/(double)m->njmax);

    // add Energy if enabled
    if( mjENABLED(mjENBL_ENERGY) )
    {
        sprintf(tmp, "\n%.3f", d->energy[0]+d->energy[1]);
        strcat(content, tmp);
        strcat(title, "\nEnergy");
    }

    // add FwdInv if enabled
    if( mjENABLED(mjENBL_FWDINV) )
    {
        sprintf(tmp, "\n%.1f %.1f",
            mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[0])),
            mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[1])));
        strcat(content, tmp);
        strcat(title, "\nFwdInv");
    }
}

void XBotMjSimEnv::printfield(char* str, void* ptr) {
    sprintf(str, "%g", *(mjtNum*)ptr);
}

void XBotMjSimEnv::watch() {
    // clear
    ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
    strcpy(ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

    // prepare constants for NC
    int nv = m->nv;
    int njmax = m->njmax;

    // find specified field in mjData arrays, update value
    #define X(TYPE, NAME, NR, NC)                                           \
        if( !strcmp(#NAME, settings.field) && !strcmp(#TYPE, "mjtNum") )    \
        {                                                                   \
            if( settings.index>=0 && settings.index<m->NR*NC )              \
                printfield(ui0.sect[SECT_WATCH].item[2].multi.name[0],      \
                           d->NAME + settings.index);                       \
            else                                                            \
                strcpy(ui0.sect[SECT_WATCH].item[2].multi.name[0],          \
                       "invalid index");                                    \
            return;                                                         \
        }

        MJDATA_POINTERS
    #undef X
}

void XBotMjSimEnv::makephysics(int oldstate) {
    int i;

    mjuiDef defPhysics[] =
    {
        {mjITEM_SECTION,   "Physics",       oldstate, NULL,                 "AP"},
        {mjITEM_SELECT,    "Integrator",    2, &(instance->m->opt.integrator),        "Euler\nRK4"},
        {mjITEM_SELECT,    "Collision",     2, &(instance->m->opt.collision),         "All\nPair\nDynamic"},
        {mjITEM_SELECT,    "Cone",          2, &(instance->m->opt.cone),              "Pyramidal\nElliptic"},
        {mjITEM_SELECT,    "Jacobian",      2, &(instance->m->opt.jacobian),          "Dense\nSparse\nAuto"},
        {mjITEM_SELECT,    "Solver",        2, &(instance->m->opt.solver),            "PGS\nCG\nNewton"},
        {mjITEM_SEPARATOR, "Algorithmic Parameters", 1},
        {mjITEM_EDITNUM,   "Timestep",      2, &(instance->m->opt.timestep),          "1 0 1"},
        {mjITEM_EDITINT,   "Iterations",    2, &(instance->m->opt.iterations),        "1 0 1000"},
        {mjITEM_EDITNUM,   "Tolerance",     2, &(instance->m->opt.tolerance),         "1 0 1"},
        {mjITEM_EDITINT,   "Noslip Iter",   2, &(instance->m->opt.noslip_iterations), "1 0 1000"},
        {mjITEM_EDITNUM,   "Noslip Tol",    2, &(instance->m->opt.noslip_tolerance),  "1 0 1"},
        {mjITEM_EDITINT,   "MRR Iter",      2, &(instance->m->opt.mpr_iterations),    "1 0 1000"},
        {mjITEM_EDITNUM,   "MPR Tol",       2, &(instance->m->opt.mpr_tolerance),     "1 0 1"},
        {mjITEM_EDITNUM,   "API Rate",      2, &(instance->m->opt.apirate),           "1 0 1000"},
        {mjITEM_SEPARATOR, "Physical Parameters", 1},
        {mjITEM_EDITNUM,   "Gravity",       2, instance->m->opt.gravity,              "3"},
        {mjITEM_EDITNUM,   "Wind",          2, instance->m->opt.wind,                 "3"},
        {mjITEM_EDITNUM,   "Magnetic",      2, instance->m->opt.magnetic,             "3"},
        {mjITEM_EDITNUM,   "Density",       2, &(instance->m->opt.density),           "1"},
        {mjITEM_EDITNUM,   "Viscosity",     2, &(instance->m->opt.viscosity),         "1"},
        {mjITEM_EDITNUM,   "Imp Ratio",     2, &(instance->m->opt.impratio),          "1"},
        {mjITEM_SEPARATOR, "Disable Flags", 1},
        {mjITEM_END}
    };
    mjuiDef defEnableFlags[] =
    {
        {mjITEM_SEPARATOR, "Enable Flags", 1},
        {mjITEM_END}
    };
    mjuiDef defOverride[] =
    {
        {mjITEM_SEPARATOR, "Contact Override", 1},
        {mjITEM_EDITNUM,   "Margin",        2, &(instance->m->opt.o_margin),          "1"},
        {mjITEM_EDITNUM,   "Sol Imp",       2, &(instance->m->opt.o_solimp),          "5"},
        {mjITEM_EDITNUM,   "Sol Ref",       2, &(instance->m->opt.o_solref),          "2"},
        {mjITEM_END}
    };

    // add physics
    mjui_add(&instance->ui0, defPhysics);

    // add flags programmatically
    mjuiDef defFlag[] =
    {
        {mjITEM_CHECKINT,  "", 2, NULL, ""},
        {mjITEM_END}
    };
    for( i=0; i<mjNDISABLE; i++ )
    {
        strcpy(defFlag[0].name, mjDISABLESTRING[i]);
        defFlag[0].pdata = settings.disable + i;
        mjui_add(&instance->ui0, defFlag);
    }
    mjui_add(&instance->ui0, defEnableFlags);
    for( i=0; i<mjNENABLE; i++ )
    {
        strcpy(defFlag[0].name, mjENABLESTRING[i]);
        defFlag[0].pdata = settings.enable + i;
        mjui_add(&instance->ui0, defFlag);
    }

    // add contact override
    mjui_add(&instance->ui0, defOverride);
}

void XBotMjSimEnv::makerendering(int oldstate) {
    int i, j;

    mjuiDef defRendering[] =
    {
        {mjITEM_SECTION,    "Rendering",        oldstate, NULL,             "AR"},
        {mjITEM_SELECT,     "Camera",           2, &(settings.camera),      "Free\nTracking"},
        {mjITEM_SELECT,     "Label",            2, &(instance->instance->vopt.label),
            "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce"},
        {mjITEM_SELECT,     "Frame",            2, &(instance->vopt.frame),
            "None\nBody\nGeom\nSite\nCamera\nLight\nWorld"},
        {mjITEM_SEPARATOR,  "Model Elements",   1},
        {mjITEM_END}
    };
    mjuiDef defOpenGL[] =
    {
        {mjITEM_SEPARATOR, "OpenGL Effects", 1},
        {mjITEM_END}
    };

    // add model cameras, up to UI limit
    for( i=0; i<mjMIN(instance->m->ncam, mjMAXUIMULTI-2); i++ )
    {
        // prepare name
        char camname[mjMAXUITEXT] = "\n";
        if( instance->m->names[instance->m->name_camadr[i]] )
            strcat(camname, instance->m->names+instance->m->name_camadr[i]);
        else
            sprintf(camname, "\nCamera %d", i);

        // check string length
        if( strlen(camname) + strlen(defRendering[1].other)>=mjMAXUITEXT-1 )
            break;

        // add camera
        strcat(defRendering[1].other, camname);
    }

    // add rendering standard
    mjui_add(&instance->ui0, defRendering);

    // add flags programmatically
    mjuiDef defFlag[] =
    {
        {mjITEM_CHECKBYTE,  "", 2, NULL, ""},
        {mjITEM_END}
    };
    for( i=0; i<mjNVISFLAG; i++ )
    {
        // set name, remove "&"
        strcpy(defFlag[0].name, mjVISSTRING[i][0]);
        for( j=0; j<strlen(mjVISSTRING[i][0]); j++ )
            if( mjVISSTRING[i][0][j]=='&' )
            {
                strcpy(defFlag[0].name+j, mjVISSTRING[i][0]+j+1);
                break;
            }

        // set shortcut and data
        sprintf(defFlag[0].other, " %s", mjVISSTRING[i][2]);
        defFlag[0].pdata = instance->vopt.flags + i;
        mjui_add(&instance->ui0, defFlag);
    }
    mjui_add(&instance->ui0, defOpenGL);
    for( i=0; i<mjNRNDFLAG; i++ )
    {
        strcpy(defFlag[0].name, mjRNDSTRING[i][0]);
        sprintf(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
        defFlag[0].pdata = instance->scn.flags + i;
        mjui_add(&instance->ui0, defFlag);
    }
}

void XBotMjSimEnv::makegroup(int oldstate) {
    mjuiDef defGroup[] =
    {
        {mjITEM_SECTION,    "Group enable",     oldstate, NULL,             "AG"},
        {mjITEM_SEPARATOR,  "Geom groups",  1},
        {mjITEM_CHECKBYTE,  "Geom 0",           2, instance->vopt.geomgroup,          " 0"},
        {mjITEM_CHECKBYTE,  "Geom 1",           2, instance->vopt.geomgroup+1,        " 1"},
        {mjITEM_CHECKBYTE,  "Geom 2",           2, instance->vopt.geomgroup+2,        " 2"},
        {mjITEM_CHECKBYTE,  "Geom 3",           2, instance->vopt.geomgroup+3,        " 3"},
        {mjITEM_CHECKBYTE,  "Geom 4",           2, instance->vopt.geomgroup+4,        " 4"},
        {mjITEM_CHECKBYTE,  "Geom 5",           2, instance->vopt.geomgroup+5,        " 5"},
        {mjITEM_SEPARATOR,  "Site groups",  1},
        {mjITEM_CHECKBYTE,  "Site 0",           2, instance->vopt.sitegroup,          "S0"},
        {mjITEM_CHECKBYTE,  "Site 1",           2, instance->vopt.sitegroup+1,        "S1"},
        {mjITEM_CHECKBYTE,  "Site 2",           2, instance->vopt.sitegroup+2,        "S2"},
        {mjITEM_CHECKBYTE,  "Site 3",           2, instance->vopt.sitegroup+3,        "S3"},
        {mjITEM_CHECKBYTE,  "Site 4",           2, instance->vopt.sitegroup+4,        "S4"},
        {mjITEM_CHECKBYTE,  "Site 5",           2, instance->vopt.sitegroup+5,        "S5"},
        {mjITEM_SEPARATOR,  "Joint groups", 1},
        {mjITEM_CHECKBYTE,  "Joint 0",          2, instance->vopt.jointgroup,         ""},
        {mjITEM_CHECKBYTE,  "Joint 1",          2, instance->vopt.jointgroup+1,       ""},
        {mjITEM_CHECKBYTE,  "Joint 2",          2, instance->vopt.jointgroup+2,       ""},
        {mjITEM_CHECKBYTE,  "Joint 3",          2, instance->vopt.jointgroup+3,       ""},
        {mjITEM_CHECKBYTE,  "Joint 4",          2, instance->vopt.jointgroup+4,       ""},
        {mjITEM_CHECKBYTE,  "Joint 5",          2, instance->vopt.jointgroup+5,       ""},
        {mjITEM_SEPARATOR,  "Tendon groups",    1},
        {mjITEM_CHECKBYTE,  "Tendon 0",         2, instance->vopt.tendongroup,        ""},
        {mjITEM_CHECKBYTE,  "Tendon 1",         2, instance->vopt.tendongroup+1,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 2",         2, instance->vopt.tendongroup+2,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 3",         2, instance->vopt.tendongroup+3,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 4",         2, instance->vopt.tendongroup+4,      ""},
        {mjITEM_CHECKBYTE,  "Tendon 5",         2, instance->vopt.tendongroup+5,      ""},
        {mjITEM_SEPARATOR,  "Actuator groups", 1},
        {mjITEM_CHECKBYTE,  "Actuator 0",       2, instance->vopt.actuatorgroup,      ""},
        {mjITEM_CHECKBYTE,  "Actuator 1",       2, instance->vopt.actuatorgroup+1,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 2",       2, instance->vopt.actuatorgroup+2,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 3",       2, instance->vopt.actuatorgroup+3,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 4",       2, instance->vopt.actuatorgroup+4,    ""},
        {mjITEM_CHECKBYTE,  "Actuator 5",       2, instance->vopt.actuatorgroup+5,    ""},
        {mjITEM_END}
    };

    // add section
    mjui_add(&instance->ui0, defGroup);
}

void XBotMjSimEnv::makejoint(int oldstate) {
    int i;

    mjuiDef defJoint[] =
    {
        {mjITEM_SECTION, "Joint", oldstate, NULL, "AJ"},
        {mjITEM_END}
    };
    mjuiDef defSlider[] =
    {
        {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
        {mjITEM_END}
    };

    // add section
    mjui_add(&instance->ui1, defJoint);
    defSlider[0].state = 4;

    // add scalar joints, exit if UI limit reached
    int itemcnt = 0;
    for( i=0; i<instance->m->njnt && itemcnt<mjMAXUIITEM; i++ )
        if( (instance->m->jnt_type[i]==mjJNT_HINGE || instance->m->jnt_type[i]==mjJNT_SLIDE) )
        {
            // skip if joint group is disabled
            if( !instance->vopt.jointgroup[mjMAX(0, mjMIN(mjNGROUP-1, instance->m->jnt_group[i]))] )
                continue;

            // set data and name
            defSlider[0].pdata = instance->d->qpos + instance->m->jnt_qposadr[i];
            if( instance->m->names[instance->m->name_jntadr[i]] )
                mju_strncpy(defSlider[0].name, instance->m->names+instance->m->name_jntadr[i],
                            mjMAXUINAME);
            else
                sprintf(defSlider[0].name, "joint %d", i);

            // set range
            if( instance->m->jnt_limited[i] )
                sprintf(defSlider[0].other, "%.4g %.4g",
                    instance->m->jnt_range[2*i], instance->m->jnt_range[2*i+1]);
            else if( instance->m->jnt_type[i]==mjJNT_SLIDE )
                strcpy(defSlider[0].other, "-1 1");
            else
                strcpy(defSlider[0].other, "-3.1416 3.1416");

            // add and count
            mjui_add(&instance->ui1, defSlider);
            itemcnt++;
        }
}

void XBotMjSimEnv::makecontrol(int oldstate) {
    int i;

    mjuiDef defControl[] =
    {
        {mjITEM_SECTION, "Control", oldstate, NULL, "AC"},
        {mjITEM_BUTTON,  "Clear all", 2},
        {mjITEM_END}
    };
    mjuiDef defSlider[] =
    {
        {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
        {mjITEM_END}
    };

    // add section
    mjui_add(&instance->ui1, defControl);
    defSlider[0].state = 2;

    // add controls, exit if UI limit reached (Clear button already added)
    int itemcnt = 1;
    for( i=0; i<instance->m->nu && itemcnt<mjMAXUIITEM; i++ )
    {
        // skip if actuator group is disabled
        if( !instance->vopt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP-1, instance->m->actuator_group[i]))] )
            continue;

        // set data and name
        defSlider[0].pdata = instance->d->ctrl + i;
        if( instance->m->names[instance->m->name_actuatoradr[i]] )
            mju_strncpy(defSlider[0].name, instance->m->names+instance->m->name_actuatoradr[i],
                        mjMAXUINAME);
        else
            sprintf(defSlider[0].name, "control %d", i);

        // set range
        if( instance->m->actuator_ctrllimited[i] )
            sprintf(defSlider[0].other, "%.4g %.4g",
                instance->m->actuator_ctrlrange[2*i], instance->m->actuator_ctrlrange[2*i+1]);
        else
            strcpy(defSlider[0].other, "-1 1");

        // add and count
        mjui_add(&instance->ui1, defSlider);
        itemcnt++;
    }
}

void XBotMjSimEnv::makesections() {
    int i;

    // get section open-close state, UI 0
    int oldstate0[NSECT0];
    for( i=0; i<NSECT0; i++ )
    {
        oldstate0[i] = 0;
        if( instance->ui0.nsect>i )
            oldstate0[i] = instance->ui0.sect[i].state;
    }

    // get section open-close state, UI 1
    int oldstate1[NSECT1];
    for( i=0; i<NSECT1; i++ )
    {
        oldstate1[i] = 0;
        if( instance->ui1.nsect>i )
            oldstate1[i] = instance->ui1.sect[i].state;
    }

    // clear model-dependent sections of UI
    instance->ui0.nsect = SECT_PHYSICS;
    instance->ui1.nsect = 0;

    // make
    makephysics(oldstate0[SECT_PHYSICS]);
    makerendering(oldstate0[SECT_RENDERING]);
    makegroup(oldstate0[SECT_GROUP]);
    makejoint(oldstate1[SECT_JOINT]);
    makecontrol(oldstate1[SECT_CONTROL]);
}

int XBotMjSimEnv::uiPredicate(int category, void* userdata)
{
    switch( category )
    {
    case 2:                 // require model
        return (instance->m!=NULL);

    case 3:                 // require model and nkey
        return (instance->m && instance->m->nkey);

    case 4:                 // require model and paused
        return (instance->m && !settings.run);

    default:
        return 1;
    }
}

void XBotMjSimEnv::uiLayout(mjuiState* state)
{
    mjrRect* rect = state->rect;

    // set number of rectangles
    state->nrect = 4;

    // rect 0: entire framebuffer
    rect[0].left = 0;
    rect[0].bottom = 0;
    glfwGetFramebufferSize(instance->window, &rect[0].width, &rect[0].height);

    // rect 1: UI 0
    rect[1].left = 0;
    rect[1].width = settings.ui0 ? instance->ui0.width : 0;
    rect[1].bottom = 0;
    rect[1].height = rect[0].height;

    // rect 2: UI 1
    rect[2].width = settings.ui1 ? instance->ui1.width : 0;
    rect[2].left = mjMAX(0, rect[0].width - rect[2].width);
    rect[2].bottom = 0;
    rect[2].height = rect[0].height;

    // rect 3: 3D plot (everything else is an overlay)
    rect[3].left = rect[1].width;
    rect[3].width = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
    rect[3].bottom = 0;
    rect[3].height = rect[0].height;
}

void XBotMjSimEnv::uiEvent(mjuiState* state)
{
    int i;
    char err[200];

    // call UI 0 if event is directed to it
    if( (state->dragrect==instance->ui0.rectid) ||
        (state->dragrect==0 && state->mouserect==instance->ui0.rectid) ||
        state->type==mjEVENT_KEY )
    {
        // process UI event
        mjuiItem* it = mjui_event(&instance->ui0, state, &instance->con);

        // file section
        if( it && it->sectionid==SECT_FILE )
        {
            switch( it->itemid )
            {
            case 0:             // Save xml
                if( !mj_saveLastXML("mjmodel.xml", instance->m, err, 200) )
                    printf("Save XML error: %s", err);
                break;

            case 1:             // Save mjb
                mj_saveModel(instance->m, "mjmodel.mjb", NULL, 0);
                break;

            case 2:             // Print model
                mj_printModel(instance->m, "MJMODEL.TXT");
                break;

            case 3:             // Print data
                mj_printData(instance->m, instance->d, "MJDATA.TXT");
                break;

            case 4:             // Quit
                settings.exitrequest = 1;
                break;
            }
        }

        // option section
        else if( it && it->sectionid==SECT_OPTION )
        {
            switch( it->itemid )
            {
            case 0:             // Spacing
                instance->ui0.spacing = mjui_themeSpacing(settings.spacing);
                instance->ui1.spacing = mjui_themeSpacing(settings.spacing);
                break;

            case 1:             // Color
                instance->ui0.color = mjui_themeColor(settings.color);
                instance->ui1.color = mjui_themeColor(settings.color);
                break;

            case 2:             // Font
                mjr_changeFont(50*(settings.font+1), &instance->con);
                break;

            case 9:             // Full screen
                if( glfwGetWindowMonitor(instance->window) )
                {
                    // restore window from saved data
                    glfwSetWindowMonitor(instance->window, NULL, instance->windowpos[0], instance->windowpos[1],
                        instance->windowsize[0], instance->windowsize[1], 0);
                }

                // currently windowed: switch to full screen
                else
                {
                    // save window data
                    glfwGetWindowPos(instance->window, instance->windowpos, instance->windowpos+1);
                    glfwGetWindowSize(instance->window, instance->windowsize, instance->windowsize+1);

                    // switch
                    glfwSetWindowMonitor(instance->window, glfwGetPrimaryMonitor(), 0, 0,
                        instance->vmode.width, instance->vmode.height, instance->vmode.refreshRate);
                }

                // reinstante vsync, just in case
                glfwSwapInterval(settings.vsync);
                break;

            case 10:            // Vertical sync
                glfwSwapInterval(settings.vsync);
                break;
            }

            // modify UI
            uiModify(instance->window, &instance->ui0, state, &instance->con);
            uiModify(instance->window, &instance->ui1, state, &instance->con);
        }

        // simulation section
        else if( it && it->sectionid==SECT_SIMULATION )
        {
            switch( it->itemid )
            {
            case 1:             // Reset
                if( instance->m )
                {
                    mj_resetData(instance->m, instance->d);
                    mj_forward(instance->m, instance->d);
                    instance->profilerupdate();
                    instance->sensorupdate();
                    instance->updateSettings();
                }
                break;

            case 2:             // Reload
                settings.loadrequest = 1;
                break;

            case 3:             // Align
                instance->alignScale();
                instance->updateSettings();
                break;

            case 4:             // Copy pose
                instance->copyKey();
                break;

            case 5:             // Adjust key
            case 6:             // Reset to key
                i = settings.key;
                instance->d->time = instance->m->key_time[i];
                mju_copy(instance->d->qpos, instance->m->key_qpos+i*instance->m->nq, instance->m->nq);
                mju_copy(instance->d->qvel, instance->m->key_qvel+i*instance->m->nv, instance->m->nv);
                mju_copy(instance->d->act, instance->m->key_act+i*instance->m->na, instance->m->na);
                mju_copy(instance->d->mocap_pos, instance->m->key_mpos+i*3*instance->m->nmocap, 3*instance->m->nmocap);
                mju_copy(instance->d->mocap_quat, instance->m->key_mquat+i*4*instance->m->nmocap, 4*instance->m->nmocap);
                mj_forward(instance->m, instance->d);
                instance->profilerupdate();
                instance->sensorupdate();
                instance->updateSettings();
                break;

            case 7:             // Set key
                i = settings.key;
                instance->m->key_time[i] = instance->d->time;
                mju_copy(instance->m->key_qpos+i*instance->m->nq, instance->d->qpos, instance->m->nq);
                mju_copy(instance->m->key_qvel+i*instance->m->nv, instance->d->qvel, instance->m->nv);
                mju_copy(instance->m->key_act+i*instance->m->na, instance->d->act, instance->m->na);
                mju_copy(instance->m->key_mpos+i*3*instance->m->nmocap, instance->d->mocap_pos, 3*instance->m->nmocap);
                mju_copy(instance->m->key_mquat+i*4*instance->m->nmocap, instance->d->mocap_quat, 4*instance->m->nmocap);
                break;
            }
        }

        // physics section
        else if( it && it->sectionid==SECT_PHYSICS )
        {
            // update disable flags in mjOption
            instance->m->opt.disableflags = 0;
            for( i=0; i<mjNDISABLE; i++ )
                if( settings.disable[i] )
                    instance->m->opt.disableflags |= (1<<i);

            // update enable flags in mjOption
            instance->m->opt.enableflags = 0;
            for( i=0; i<mjNENABLE; i++ )
                if( settings.enable[i] )
                    instance->m->opt.enableflags |= (1<<i);
        }

        // rendering section
        else if( it && it->sectionid==SECT_RENDERING )
        {
            // set camera in mjvCamera
            if( settings.camera==0 )
                instance->cam.type = mjCAMERA_FREE;
            else if( settings.camera==1 )
            {
                if( instance->pert.select>0 )
                {
                    instance->cam.type = mjCAMERA_TRACKING;
                    instance->cam.trackbodyid = instance->pert.select;
                    instance->cam.fixedcamid = -1;
                }
                else
                {
                    instance->cam.type = mjCAMERA_FREE;
                    settings.camera = 0;
                    mjui_update(SECT_RENDERING, -1, &instance->ui0, &instance->uistate, &instance->con);
                }
            }
            else
            {
                instance->cam.type = mjCAMERA_FIXED;
                instance->cam.fixedcamid = settings.camera - 2;
            }
        }

        // group section
        else if( it && it->sectionid==SECT_GROUP )
        {
            // remake joint section if joint group changed
            if( it->name[0]=='J' && it->name[1]=='o' )
            {
                instance->ui1.nsect = SECT_JOINT;
                makejoint(instance->ui1.sect[SECT_JOINT].state);
                instance->ui1.nsect = NSECT1;
                uiModify(instance->window, &instance->ui1, state, &instance->con);
            }

            // remake control section if actuator group changed
            if( it->name[0]=='A' && it->name[1]=='c' )
            {
                instance->ui1.nsect = SECT_CONTROL;
                makecontrol(instance->ui1.sect[SECT_CONTROL].state);
                instance->ui1.nsect = NSECT1;
                uiModify(instance->window, &instance->ui1, state, &instance->con);
            }
        }

        // stop if UI processed event
        if( it!=NULL || (state->type==mjEVENT_KEY && state->key==0) )
            return;
    }

    // call UI 1 if event is directed to it
    if( (state->dragrect==instance->ui1.rectid) ||
        (state->dragrect==0 && state->mouserect==instance->ui1.rectid) ||
        state->type==mjEVENT_KEY )
    {
        // process UI event
        mjuiItem* it = mjui_event(&instance->ui1, state, &instance->con);

        // control section
        if( it && it->sectionid==SECT_CONTROL )
        {
            // clear controls
            if( it->itemid==0 )
            {
                mju_zero(instance->d->ctrl, instance->m->nu);
                mjui_update(SECT_CONTROL, -1, &instance->ui1, &instance->uistate, &instance->con);
            }
        }

        // stop if UI processed event
        if( it!=NULL || (state->type==mjEVENT_KEY && state->key==0) )
            return;
    }

    // shortcut not handled by UI
    if( state->type==mjEVENT_KEY && state->key!=0 )
    {
        switch( state->key )
        {
        case ' ':                   // Mode
            if( instance->m )
            {
                settings.run = 1 - settings.run;
                instance->pert.active = 0;
                mjui_update(-1, -1, &instance->ui0, state, &instance->con);
            }
            break;

        case mjKEY_RIGHT:           // step forward
            if( instance->m && !settings.run )
            {
                instance->clearTimers();
                mj_step(instance->m, instance->d);
                instance->profilerupdate();
                instance->sensorupdate();
                instance->updateSettings();
            }
            break;

        case mjKEY_LEFT:            // step back
            if( instance->m && !settings.run )
            {
                instance->m->opt.timestep = -instance->m->opt.timestep;
                instance->clearTimers();
                mj_step(instance->m, instance->d);
                instance->m->opt.timestep = -instance->m->opt.timestep;
                instance->profilerupdate();
                instance->sensorupdate();
                instance->updateSettings();
            }
            break;

        case mjKEY_DOWN:            // step forward 100
            if( instance->m && !settings.run )
            {
                instance->clearTimers();
                for( i=0; i<100; i++ )
                    mj_step(instance->m, instance->d);
                instance->profilerupdate();
                instance->sensorupdate();
                instance->updateSettings();
            }
            break;

        case mjKEY_UP:              // step back 100
            if( instance->m && !settings.run )
            {
                instance->m->opt.timestep = -instance->m->opt.timestep;
                instance->clearTimers();
                for( i=0; i<100; i++ )
                    mj_step(instance->m, instance->d);
                instance->m->opt.timestep = -instance->m->opt.timestep;
                instance->profilerupdate();
                instance->sensorupdate();
                instance->updateSettings();
            }
            break;

        case mjKEY_PAGE_UP:         // select parent body
            if( instance->m && instance->pert.select>0 )
            {
                instance->pert.select = instance->m->body_parentid[instance->pert.select];
                instance->pert.skinselect = -1;

                // stop perturbation if world reached
                if( instance->pert.select<=0 )
                    instance->pert.active = 0;
            }

            break;

        case mjKEY_ESCAPE:          // free camera
            instance->cam.type = mjCAMERA_FREE;
            settings.camera = 0;
            mjui_update(SECT_RENDERING, -1, &instance->ui0, &instance->uistate, &instance->con);
            break;
        }

        return;
    }

    // 3D scroll
    if( state->type==mjEVENT_SCROLL && state->mouserect==3 && instance->m )
    {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(instance->m, mjMOUSE_ZOOM, 0, -0.05*state->sy, &instance->scn, &instance->cam);

        return;
    }

    // 3D press
    if( state->type==mjEVENT_PRESS && state->mouserect==3 && instance->m )
    {
        // set perturbation
        int newperturb = 0;
        if( state->control && instance->pert.select>0 )
        {
            // right: translate;  left: rotate
            if( state->right )
                newperturb = mjPERT_TRANSLATE;
            else if( state->left )
                newperturb = mjPERT_ROTATE;

            // perturbation onset: reset reference
            if( newperturb && !instance->pert.active )
                mjv_initPerturb(instance->m, instance->d, &instance->scn, &instance->pert);
        }
        instance->pert.active = newperturb;

        // handle double-click
        if( state->doubleclick )
        {
            // determine selection mode
            int selmode;
            if( state->button==mjBUTTON_LEFT )
                selmode = 1;
            else if( state->control )
                selmode = 3;
            else
                selmode = 2;

            // find geom and 3D click point, get corresponding body
            mjrRect r = state->rect[3];
            mjtNum selpnt[3];
            int selgeom, selskin;
            int selbody = mjv_select(instance->m, instance->d, &instance->vopt,
                                     (mjtNum)r.width/(mjtNum)r.height,
                                     (mjtNum)(state->x-r.left)/(mjtNum)r.width,
                                     (mjtNum)(state->y-r.bottom)/(mjtNum)r.height,
                                     &instance->scn, selpnt, &selgeom, &selskin);

            // set lookat point, start tracking is requested
            if( selmode==2 || selmode==3 )
            {
                // copy selpnt if anything clicked
                if( selbody>=0 )
                    mju_copy3(instance->cam.lookat, selpnt);

                // switch to tracking camera if dynamic body clicked
                if( selmode==3 && selbody>0 )
                {
                    // mujoco camera
                    instance->cam.type = mjCAMERA_TRACKING;
                    instance->cam.trackbodyid = selbody;
                    instance->cam.fixedcamid = -1;

                    // UI camera
                    settings.camera = 1;
                    mjui_update(SECT_RENDERING, -1, &instance->ui0, &instance->uistate, &instance->con);
                }
            }

            // set body selection
            else
            {
                if( selbody>=0 )
                {
                    // record selection
                    instance->pert.select = selbody;
                    instance->pert.skinselect = selskin;

                    // compute localpos
                    mjtNum tmp[3];
                    mju_sub3(tmp, selpnt, instance->d->xpos+3*instance->pert.select);
                    mju_mulMatTVec(instance->pert.localpos, instance->d->xmat+9*instance->pert.select, tmp, 3, 3);
                }
                else
                {
                    instance->pert.select = 0;
                    instance->pert.skinselect = -1;
                }
            }

            // stop perturbation on select
            instance->pert.active = 0;
        }

        return;
    }

    // 3D release
    if( state->type==mjEVENT_RELEASE && state->dragrect==3 && instance->m )
    {
        // stop perturbation
        instance->pert.active = 0;

        return;
    }

    // 3D move
    if( state->type==mjEVENT_MOVE && state->dragrect==3 && instance->m )
    {
        // determine action based on mouse button
        mjtMouse action;
        if( state->right )
            action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if( state->left )
            action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move perturb or camera
        mjrRect r = state->rect[3];
        if( instance->pert.active )
            mjv_movePerturb(instance->m, instance->d, action, state->dx/r.height, -state->dy/r.height,
                            &instance->scn, &instance->pert);
        else
            mjv_moveCamera(instance->m, action, state->dx/r.height, -state->dy/r.height,
                           &instance->scn, &instance->cam);

        return;
    }
}

void XBotMjSimEnv::prepare() {
    lastUpdateTime = 0;
    double currentTime = glfwGetTime();
    double interval = currentTime - lastUpdateTime;
    interval = mjMIN(1, mjMAX(0.0001, interval));
    lastUpdateTime = currentTime;

    // no model: nothing to do
    if( !m )
        return;

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // update watch
    if( settings.ui0 && ui0.sect[SECT_WATCH].state )
    {
        watch();
        mjui_update(SECT_WATCH, -1, &ui0, &uistate, &con);
    }

    // ipdate joint
    if( settings.ui1 && ui1.sect[SECT_JOINT].state )
            mjui_update(SECT_JOINT, -1, &ui1, &uistate, &con);

    // update info text
    if( settings.info )
        infotext(info_title, info_content, interval);

    // update profiler
    if( settings.profiler && settings.run )
        profilerupdate();

    // update sensor
    if( settings.sensor && settings.run )
        sensorupdate();

    // clear timers once profiler info has been copied
    clearTimers();
}

void XBotMjSimEnv::render(GLFWwindow* window) {
    // get 3D rectangle and reduced for profiler
    mjrRect rect = instance->uistate.rect[3];
    mjrRect smallrect = rect;
    if( settings.profiler )
        smallrect.width = rect.width - rect.width/4;

    // no model
    if( !instance->m )
    {
        // blank screen
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

        // label
        if( settings.loadrequest )
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                        "loading", NULL, &instance->con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect,
                        "Drag-and-drop model file here", 0, &instance->con);

        // render uis
        if( settings.ui0 )
            mjui_render(&instance->ui0, &instance->uistate, &instance->con);
        if( settings.ui1 )
            mjui_render(&instance->ui1, &instance->uistate, &instance->con);

        // finalize
        glfwSwapBuffers(window);

        return;
    }

    // render scene
    mjr_render(rect, &instance->scn, &instance->con);

    // show pause/loading label
    if( !settings.run || settings.loadrequest )
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                    settings.loadrequest ? "loading" : "pause", NULL, &instance->con);

    // show ui 0
    if( settings.ui0 )
        mjui_render(&instance->ui0, &instance->uistate, &instance->con);

    // show ui 1
    if( settings.ui1 )
        mjui_render(&instance->ui1, &instance->uistate, &instance->con);

    // show help
    if( settings.help )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &instance->con);

    // show info
    if( settings.info )
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect,
                    info_title, info_content, &instance->con);

    // show profiler
    if( settings.profiler )
        profilershow(rect);

    // show sensor
    if( settings.sensor )
        sensorshow(smallrect);

    // finalize
    glfwSwapBuffers(window);
}

void XBotMjSimEnv::simulate() {
    // cpu-sim syncronization point
    double cpusync = 0;
    mjtNum simsync = 0;

    // run until asked to exit
    while(!settings.exitrequest)
    {
        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if(settings.run && settings.busywait)
            std::this_thread::yield();
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // start exclusive access
        mtx.lock();

        // run only if model is present
        if(m)
        {
            // record start time
            double startwalltm = glfwGetTime();

            // running
            if(settings.run)
            {
                // record cpu time at start of iteration
                double tmstart = glfwGetTime();

                // out-of-sync (for any reason)
                if( d->time<simsync || tmstart<cpusync || cpusync==0 ||
                    mju_abs((d->time-simsync)-(tmstart-cpusync))>syncmisalign )
                {
                    // re-sync
                    cpusync = tmstart;
                    simsync = d->time;

                    // clear old perturbations, apply new
                    mju_zero(d->xfrc_applied, 6*m->nbody);
                    mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
                    mjv_applyPerturbForce(m, d, &pert);

                    // run single step, let next iteration deal with timing
                    mj_step(m, d);
                }

                // in-sync
                else
                {
                    // step while simtime lags behind cputime, and within safefactor
                    while( (d->time-simsync)<(glfwGetTime()-cpusync) &&
                           (glfwGetTime()-tmstart)<refreshfactor/vmode.refreshRate )
                    {
                        // clear old perturbations, apply new
                        mju_zero(d->xfrc_applied, 6*m->nbody);
                        mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
                        mjv_applyPerturbForce(m, d, &pert);

                        // run mj_step
                        mjtNum prevtm = d->time;
                        mj_step(m, d);

                        // break on reset
                        if( d->time<prevtm )
                            break;
                    }
                }
            }

            // paused
            else
            {
                // apply pose perturbation
                mjv_applyPerturbPose(m, d, &pert, 1);      // move mocap and dynamic bodies

                // run mj_forward, to update rendering and joint sliders
                mj_forward(m, d);
            }
        }

        // end exclusive access
        mtx.unlock();
    }
}

void XBotMjSimEnv::alignScale() {
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
    cam.type = mjCAMERA_FREE;
}

void XBotMjSimEnv::copyKey() {
    char clipboard[5000] = "<key qpos='";
    char buf[200];

    // prepare string
    for( int i=0; i<m->nq; i++ )
    {
        sprintf(buf, i==m->nq-1 ? "%g" : "%g ", d->qpos[i]);
        strcat(clipboard, buf);
    }
    strcat(clipboard, "'/>");

    // copy to clipboard
    glfwSetClipboardString(window, clipboard);
}

mjtNum XBotMjSimEnv::timer() {
    return (mjtNum)(1000 * glfwGetTime());
}

void XBotMjSimEnv::clearTimers() {
    for (int i = 0; i < mjNTIMER; i++) {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

void XBotMjSimEnv::updateSettings() {
    int i;

    // physics flags
    for( i=0; i<mjNDISABLE; i++ )
        settings.disable[i] = ((m->opt.disableflags & (1<<i)) !=0 );
    for( i=0; i<mjNENABLE; i++ )
        settings.enable[i] = ((m->opt.enableflags & (1<<i)) !=0 );

    // camera
    if( cam.type==mjCAMERA_FIXED )
        settings.camera = 2 + cam.fixedcamid;
    else if( cam.type==mjCAMERA_TRACKING )
        settings.camera = 1;
    else
        settings.camera = 0;

    // update UI
    mjui_update(-1, -1, &ui0, &uistate, &con);
}

void XBotMjSimEnv::drop(GLFWwindow* window, int count, const char** paths) {
    if (count > 0) {
        mju_strncpy(instance->filename, paths[0], 1000);
        settings.loadrequest = 1;
    }
}

void XBotMjSimEnv::loadModel() {
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if( !filename[0]  )
        return;

    // load and compile
    char error[500] = "";
    mjModel* mnew = 0;
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
    {
        mnew = mj_loadModel(filename, NULL);
        if( !mnew )
            strcpy(error, "could not load binary model");
    }
    else
        mnew = mj_loadXML(filename, NULL, error, 500);
    if( !mnew )
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if( error[0] )
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
                error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    xbot2_wrapper.reset();
    m = mnew;
    d = mj_makeData(m);
    xbot2_wrapper = std::make_unique<XBot::MjWrapper>(m, xbot2_cfg_path);
    mj_forward(m, d);

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50*(settings.font+1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignScale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if( window && m->names )
    {
        char title[200] = "Simulate : ";
        strcat(title, m->names);
        glfwSetWindowTitle(window, title);
    }

    // set keyframe range and divisions
    ui0.sect[SECT_SIMULATION].item[5].slider.range[0] = 0;
    ui0.sect[SECT_SIMULATION].item[5].slider.range[1] = mjMAX(0, m->nkey - 1);
    ui0.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, m->nkey - 1);

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
    updateSettings();
}

void XBotMjSimEnv::dropCallback(GLFWwindow* window, int count, const char** paths) {
    XBotMjSimEnv* env = static_cast<XBotMjSimEnv*>(glfwGetWindowUserPointer(window));
    env->drop(window, count, paths);
}

void XBotMjSimEnv::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        XBotMjSimEnv* env = static_cast<XBotMjSimEnv*>(glfwGetWindowUserPointer(window));
        // Handle key events
        // ...
    }
}