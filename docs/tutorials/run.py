import os
from notebook import notebookapp
from pathlib import Path

os.chdir("./docs/tutorials")

# sym_link_path = "../notebooks/data"
# if not Path(sym_link_path).is_symlink():
#   os.symlink("../data", sym_link_path)

notebookapp.main()