

#ifndef MRI_SWD_VERSION_STRING

#define MRI_SWD_BRANCH "https://github.com/adamgreen/mri-swd"

#define MRI_SWD_VERSION_MAJOR       0
#define MRI_SWD_VERSION_MINOR       1
#define MRI_SWD_VERSION_BUILD       20230518
#define MRI_SWD_VERSION_SUBBUILD    0

#define MRI_SWD_STR(X) MRI_SWD_STR2(X)
#define MRI_SWD_STR2(X) #X

#define MRI_SWD_VERSION_STRING MRI_SWD_STR(MRI_SWD_VERSION_MAJOR) "." MRI_SWD_STR(MRI_SWD_VERSION_MINOR) "-" MRI_SWD_STR(MRI_SWD_VERSION_BUILD) "." MRI_SWD_STR(MRI_SWD_VERSION_SUBBUILD)

#endif
