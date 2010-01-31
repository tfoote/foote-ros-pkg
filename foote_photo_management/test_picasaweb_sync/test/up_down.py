import roslib; roslib.load_manifest('test_picasaweb_sync')

import os
import struct
import sys
import unittest
import subprocess

import rostest

import tempfile

    


class Picasaweb_SyncCommandlineTest(unittest.TestCase):
    def assert_success(self, cmd_list):
        self.assertEqual(0,subprocess.call(cmd_list))

    def test_Picasaweb_Sync_commandline_round_trip_verify(self):
        temp_directory = tempfile.mkdtemp()

        try:
            test_pkg = roslib.packages.get_pkg_dir("test_picasaweb_sync")
            source_dir = os.path.join('%s'%test_pkg, 'test', 'data')
            upload_dir = os.path.join(temp_directory, 'upload_test')
            download_dir = os.path.join(temp_directory, 'download_test')
            self.assert_success(['svn', 'export', source_dir,  upload_dir])
            with open(os.path.join("/home/tfoote", "picasaweb_sync_test_template.yaml")) as upload_template:
                text = upload_template.read()
                upload_yaml_file = tempfile.NamedTemporaryFile()
                upload_yaml_file.write(text)
                upload_yaml_file.write("local_path: %s\n"%upload_dir)
                upload_yaml_file.flush()
                self.assert_success(['rosrun', 'picasaweb_sync', 'upload.py', 'upload', upload_yaml_file.name])
            with open(os.path.join("/home/tfoote", "picasaweb_sync_test_template.yaml")) as download_template:
                text = download_template.read()
                download_yaml_file = tempfile.NamedTemporaryFile()
                download_yaml_file.write(text)
                download_yaml_file.write("local_path: %s\n"%download_dir)
                download_yaml_file.flush()
                self.assert_success(['rosrun', 'picasaweb_sync', 'upload.py',  'download', download_yaml_file.name])
            self.assert_success(['diff', '-r', upload_dir, download_dir])
            print "diff passed %s vs %s"%(upload_dir, download_dir)
        finally:
            self.assert_success(['rm', '-rf', temp_directory])
        




if __name__ == '__main__':
  rostest.unitrun('test_picasaweb_sync', 'test_commandline', Picasaweb_SyncCommandlineTest, coverage_packages=['picasaweb_sync'])  

