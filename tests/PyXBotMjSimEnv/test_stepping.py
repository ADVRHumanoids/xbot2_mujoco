import unittest

from xbot2_mujoco.PyXbotMjSimEnv import XBotMjSimEnv

class TestSimSteppingHeadless(unittest.TestCase):

    def test_xmj_env(self):

        n_steps=10000
        n_steps_done=0
        for i in range(n_steps):
            self._xmj_env.step()
            n_steps_done+=1
            print(f"[test_xmj_env]: step idx: {i}\n");

        self.assertTrue(n_steps_done==n_steps_done)

    def setUp(self):

        self._xmj_env = XBotMjSimEnv(xbot2_cfg_path="",
            model_fname="",
            headless=True,
            multithread=True)

    def tearDown(self):

        self._xmj_env.close()

class TestSimStepping(unittest.TestCase):

    def test_xmj_env(self):

        n_steps=10000
        n_steps_done=0
        for i in range(n_steps):
            self._xmj_env.step()
            n_steps_done+=1
            print(f"[test_xmj_env]: step idx: {i}\n");

        self.assertTrue(n_steps_done==n_steps_done)

    def setUp(self):

        self._xmj_env = XBotMjSimEnv(xbot2_cfg_path="",
            model_fname="",
            headless=False,
            multithread=True)

    def tearDown(self):

        self._xmj_env.close()

if __name__ == "__main__":

    unittest.main()