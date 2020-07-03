import unittest
import sys
import os

import bark.core.commons


class PickleTests(unittest.TestCase):

    def test_init_glog_v1(self):
        log_folder = os.path.abspath(os.path.join(os.getcwd(), "logs"))
        if not os.path.exists(log_folder):
          os.makedirs(log_folder)
        print("Logging into: {}".format(log_folder))
        bark.core.commons.GLogInit(sys.argv[0], log_folder, 1, False)

        bark.core.commons.do_logging()

        bark.core.commons.do_logging()

        bark.core.commons.do_logging()

        bark.core.commons.do_logging()

if __name__ == '__main__':
    unittest.main()