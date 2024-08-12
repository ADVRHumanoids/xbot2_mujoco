#ifndef MJ_SIM_ENV_SETTINGS_H
#define MJ_SIM_ENV_SETTINGS_H

#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>

//-------------------------------- global -----------------------------------------------

// constants
static const int maxgeom = 5000;           // preallocated geom array in mjvScene
static const double syncmisalign = 0.1;    // maximum time mis-alignment before re-sync
static const double refreshfactor = 0.7;   // fraction of refresh available for simulation

// UI settings not contained in MuJoCo structures
static struct {
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 0;
    int font = 0;
    int ui0 = 1;
    int ui1 = 1;
    int help = 0;
    int info = 0;
    int profiler = 0;
    int sensor = 0;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 0;

    // simulation
    int run = 1;
    int key = 0;
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 0;
} settings;

// section ids
enum
{
    // left ui
    SECT_FILE   = 0,
    SECT_OPTION,
    SECT_SIMULATION,
    SECT_WATCH,
    SECT_PHYSICS,
    SECT_RENDERING,
    SECT_GROUP,
    NSECT0,

    // right ui
    SECT_JOINT = 0,
    SECT_CONTROL,
    NSECT1
};

// info strings
static char info_title[1000];
static char info_content[1000];

// option section of UI
static const mjuiDef defOption[] =
{
    {mjITEM_SECTION,   "Option",        1, NULL,                    "AO"},
    {mjITEM_SELECT,    "Spacing",       1, &settings.spacing,       "Tight\nWide"},
    {mjITEM_SELECT,    "Color",         1, &settings.color,         "Default\nOrange\nWhite\nBlack"},
    {mjITEM_SELECT,    "Font",          1, &settings.font,          "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
    {mjITEM_CHECKINT,  "Left UI (Tab)", 1, &settings.ui0,           " #258"},
    {mjITEM_CHECKINT,  "Right UI",      1, &settings.ui1,           "S#258"},
    {mjITEM_CHECKINT,  "Help",          2, &settings.help,          " #290"},
    {mjITEM_CHECKINT,  "Info",          2, &settings.info,          " #291"},
    {mjITEM_CHECKINT,  "Profiler",      2, &settings.profiler,      " #292"},
    {mjITEM_CHECKINT,  "Sensor",        2, &settings.sensor,        " #293"},
#ifdef __APPLE__
    {mjITEM_CHECKINT,  "Fullscreen",    0, &settings.fullscreen,    " #294"},
#else
    {mjITEM_CHECKINT,  "Fullscreen",    1, &settings.fullscreen,    " #294"},
#endif
    {mjITEM_CHECKINT,  "Vertical Sync", 1, &settings.vsync,         " #295"},
    {mjITEM_CHECKINT,  "Busy Wait",     1, &settings.busywait,      " #296"},
    {mjITEM_END}
};

// simulation section of UI
static const mjuiDef defSimulation[] =
{
    {mjITEM_SECTION,   "Simulation",    1, NULL,                    "AS"},
    {mjITEM_RADIO,     "",              2, &settings.run,           "Pause\nRun"},
    {mjITEM_BUTTON,    "Reset",         2, NULL,                    " #259"},
    {mjITEM_BUTTON,    "Reload",        2, NULL,                    "CL"},
    {mjITEM_BUTTON,    "Align",         2, NULL,                    "CA"},
    {mjITEM_BUTTON,    "Copy pose",     2, NULL,                    "CC"},
    {mjITEM_SLIDERINT, "Key",           3, &settings.key,           "0 0"},
    {mjITEM_BUTTON,    "Reset to key",  3},
    {mjITEM_BUTTON,    "Set key",       3},
    {mjITEM_END}
};

// watch section of UI
static const mjuiDef defWatch[] =
{
    {mjITEM_SECTION,   "Watch",         0, NULL,                    "AW"},
    {mjITEM_EDITTXT,   "Field",         2, settings.field,          "qpos"},
    {mjITEM_EDITINT,   "Index",         2, &settings.index,         "1"},
    {mjITEM_STATIC,    "Value",         2, NULL,                    " "},
    {mjITEM_END}
};

// help strings
static const char help_content[] =
"Alt mouse button\n"
"UI right hold\n"
"UI title double-click\n"
"Space\n"
"Esc\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Page Up\n"
"Double-click\n"
"Right double-click\n"
"Ctrl Right double-click\n"
"Scroll, middle drag\n"
"Left drag\n"
"[Shift] right drag\n"
"Ctrl [Shift] drag\n"
"Ctrl [Shift] right drag";

static const char help_title[] =
"Swap left-right\n"
"Show UI shortcuts\n"
"Expand/collapse all  \n"
"Pause\n"
"Free camera\n"
"Step forward\n"
"Step back\n"
"Step forward 100\n"
"Step back 100\n"
"Select parent\n"
"Select\n"
"Center\n"
"Track camera\n"
"Zoom\n"
"View rotate\n"
"View translate\n"
"Object rotate\n"
"Object translate";

static const mjuiDef defFile[] =
{
    {mjITEM_SECTION,   "File",          1, NULL,                    "AF"},
    {mjITEM_BUTTON,    "Save xml",      2, NULL,                    ""},
    {mjITEM_BUTTON,    "Save mjb",      2, NULL,                    ""},
    {mjITEM_BUTTON,    "Print model",   2, NULL,                    "CM"},
    {mjITEM_BUTTON,    "Print data",    2, NULL,                    "CD"},
    {mjITEM_BUTTON,    "Quit",          1, NULL,                    "CQ"},
    {mjITEM_END}
};

#endif // MJ_SIM_ENV_SETTINGS_H
