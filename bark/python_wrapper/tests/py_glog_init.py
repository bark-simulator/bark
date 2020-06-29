import unittest
import sys
import os

import bark.commons


class PickleTests(unittest.TestCase):

    def test_init_glog_v1(self):
        log_folder = os.path.abspath(os.path.join(os.getcwd(), "logs"))
        if not os.path.exists(log_folder):
          os.makedirs(log_folder)
        print("Logging into: {}".format(log_folder))
        bark.commons.GLogInit(sys.argv[0], log_folder, 1, False)

        bark.commons.do_logging()

        bark.commons.do_logging()

        bark.commons.do_logging()

        bark.commons.do_logging()

if __name__ == '__main__':
    unittest.main()