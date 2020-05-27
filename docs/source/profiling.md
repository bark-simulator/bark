 Profiling using Easy Profiler
================================

## Step 1: Install Easy Profiler

Install Qt5 (Paket qt5-default), see https://wiki.ubuntuusers.de/Qt/.
Clone https://github.com/yse/easy_profiler and build easy profiler.
Then install easy profiler using `make install` and add to ld_path using `ldconfig`


## Step 2: Prepare BARK Project

Make sure to have `build:easy_profiler --linkopt='-L/usr/local/lib/ -leasy_profiler' --copt='-DBUILD_WITH_EASY_PROFILER'`in your bazel.rc file.
Include easy_profiler using `\#include <easy/profiler.h>`.
In every function, you want to profile, write `EASY_FUNCTION();` at the beginning.
Defining the functions `void profiler_startup()` and void `profiler_finish()`, for example in some utility function

```
void profiler_startup() {
  EASY_PROFILER_ENABLE;
//  profiler::startListen();
}

void profiler_finish() {
  auto blocks_written = profiler::dumpBlocksToFile("/tmp/<some example>.prof");
  LOG(INFO) << "Easy profiler blocks written: " << blocks_written;
}
```

and wrap them to Python.
In the python runtime, you then have to call them before and after the code you want to profile.


## Step 3: Run BARK

The run: `bazel run --config=easy_profiler <some example target>`
Make sure to only run a short example, otherwise the profiling dump will get too big.
Profiling Dump should now be in `/tmp/<some example>.prof`.


## Step 4: Open Dump with Easy Profiler

Go to the build directory of easy_profiler and run `bin/profiler_gui`.
You then can use the GUI to open the dump file.