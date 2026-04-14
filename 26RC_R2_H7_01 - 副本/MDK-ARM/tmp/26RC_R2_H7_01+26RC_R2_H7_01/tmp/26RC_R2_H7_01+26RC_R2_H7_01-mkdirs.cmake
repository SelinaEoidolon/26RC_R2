# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01")
  file(MAKE_DIRECTORY "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01")
endif()
file(MAKE_DIRECTORY
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/1"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01/tmp"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01/src/26RC_R2_H7_01+26RC_R2_H7_01-stamp"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01/src"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01/src/26RC_R2_H7_01+26RC_R2_H7_01-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01/src/26RC_R2_H7_01+26RC_R2_H7_01-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26RC_R2_H7_01 - 副本/MDK-ARM/tmp/26RC_R2_H7_01+26RC_R2_H7_01/src/26RC_R2_H7_01+26RC_R2_H7_01-stamp${cfgdir}") # cfgdir has leading slash
endif()
