import unittest
import glob
import os
import os.path


class NotebookTests(unittest.TestCase):
  def test_run_converted_notebook(self):
    notebook_list = glob.glob("docs/tutorials/*.ipynb")
    exclude_notebooks_list = ["02_maps", "03_Benchmarking"]
    for full_notebook_name in notebook_list:
      notebook_name_ipynb = os.path.basename(full_notebook_name)
      notebook_name = notebook_name_ipynb.split('.')[0]
      print(f"Running notebook {notebook_name}.")
      if notebook_name in exclude_notebooks_list:
        print(f"Skipping notebook {notebook_name}")
      else:
        new_py_file_name = "notebook_unittest_" + notebook_name
        output = os.system(
          "jupyter nbconvert --to script --output " + new_py_file_name + " docs/tutorials/" + notebook_name + ".ipynb")
        exec(open("docs/tutorials/" + new_py_file_name + ".py").read())


if __name__ == '__main__':
  unittest.main()
