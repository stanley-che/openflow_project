cmake_minimum_required(VERSION 3.16)
project(hybrid_of LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------- Build type ----------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------- Options ----------
option(USE_COINOR "Enable MILP with COIN-OR (Cbc/Clp/Osi/CoinUtils)" ON)
option(BUILD_APP  "Build the hybrid_of executable (main loop)"         ON)

find_package(Threads REQUIRED)

# ---------- Eigen3 (config first, then header-only fallback) ----------
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_path(EIGEN3_INCLUDE_DIR Eigen/Core
            PATHS /usr/include/eigen3 /usr/local/include/eigen3)
  if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found. Install libeigen3-dev or set EIGEN3_INCLUDE_DIR.")
  endif()
  add_library(eigen3_headers INTERFACE)
  target_include_directories(eigen3_headers INTERFACE ${EIGEN3_INCLUDE_DIR})
  set(EIGEN3_TARGET eigen3_headers)
else()
  set(EIGEN3_TARGET Eigen3::Eigen)
endif()

# ---------- nlohmann_json (config or header-only fallback) ----------
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  find_path(NLJSON_INCLUDE nlohmann/json.hpp
            PATHS /usr/include /usr/local/include)
  if(NOT NLJSON_INCLUDE)
    message(FATAL_ERROR "nlohmann_json not found. Install nlohmann-json3-dev or set NLJSON_INCLUDE.")
  endif()
  add_library(nlohmann_json_headers INTERFACE)
  target_include_directories(nlohmann_json_headers INTERFACE ${NLJSON_INCLUDE})
  set(NLJSON_TARGET nlohmann_json_headers)
else()
  set(NLJSON_TARGET nlohmann_json::nlohmann_json)
endif()

# ---- COIN-OR (optional) ----
set(COIN_LIBS "")
if(USE_COINOR)
  find_library(CBC_LIB        NAMES Cbc cbc)
  find_library(CGL_LIB        NAMES Cgl cgl)
  find_library(OSI_LIB        NAMES Osi osi)
  find_library(CLP_LIB        NAMES Clp clp)
  find_library(COINUTILS_LIB  NAMES CoinUtils coinutils)
  find_library(OSI_CLP_LIB    NAMES OsiClp osiclp)
  find_library(OSI_CBC_LIB    NAMES OsiCbc osicbc)

  if(CBC_LIB AND CGL_LIB AND OSI_LIB AND CLP_LIB AND COINUTILS_LIB)
    set(COIN_LIBS ${CBC_LIB} ${CGL_LIB} ${OSI_LIB} ${CLP_LIB} ${COINUTILS_LIB})

    if(OSI_CLP_LIB)
      list(APPEND COIN_LIBS ${OSI_CLP_LIB})
    endif()

    if(OSI_CBC_LIB)
      list(APPEND COIN_LIBS ${OSI_CBC_LIB})
    endif()

    add_compile_definitions(HAVE_COINOR=1)
  else()
    message(FATAL_ERROR
      "COIN-OR core libs not found. "
      "Install coinor-cbc coinor-clp coinor-libcbc-dev coinor-libcgl-dev "
      "coinor-libclp-dev coinor-libosi-dev coinor-libcoinutils-dev "
      "or configure with -DUSE_COINOR=OFF.")
  endif()
endif()

# ---------- Sources ----------
# 核心（不含 MILP 與 main），做成靜態庫便於重用/測試
set(CORE_SRC
  src/te_controller.cpp
  src/topo_viewer.cpp
  src/monitor.cpp
  src/forecast.cpp
)
if(EXISTS "${CMAKE_SOURCE_DIR}/src/actuator.cpp")
  list(APPEND CORE_SRC src/actuator.cpp)
endif()

add_library(hybrid_of_core STATIC ${CORE_SRC})
target_include_directories(hybrid_of_core PUBLIC src)
target_link_libraries(hybrid_of_core
  PUBLIC
    Threads::Threads
    ${EIGEN3_TARGET}
    ${NLJSON_TARGET}
)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(hybrid_of_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
if (WIN32)
  target_link_libraries(hybrid_of_core PUBLIC ws2_32)
endif()

# MILP（可選）獨立出來，其他專案也能重用
if(USE_COINOR)
  add_library(milp_te STATIC src/milp_te.cpp)
  target_include_directories(milp_te PUBLIC src)
  target_link_libraries(milp_te PRIVATE ${COIN_LIBS})
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(milp_te PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------- App executable ----------
if(BUILD_APP)
  if(EXISTS "${CMAKE_SOURCE_DIR}/src/main.cpp")
    add_executable(hybrid_of src/main.cpp)
    target_include_directories(hybrid_of PRIVATE src)
    target_link_libraries(hybrid_of PRIVATE hybrid_of_core)
    if(USE_COINOR)
      target_link_libraries(hybrid_of PRIVATE milp_te)
    endif()
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
      target_compile_options(hybrid_of PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if (WIN32)
      target_link_libraries(hybrid_of PRIVATE ws2_32)
    endif()
  else()
    message(WARNING "BUILD_APP=ON but src/main.cpp not found; skipping executable.")
  endif()
endif()

# ---------- App executable ----------
# ---------- App executable ----------
if(BUILD_APP AND EXISTS "${CMAKE_SOURCE_DIR}/src/main.cpp")
  set(APP_SOURCES
    src/main.cpp
    src/HybridSDNApp.cpp
  )

  if(NOT TARGET hybrid_of)
    add_executable(hybrid_of ${APP_SOURCES})
  else()
    # 另一處已經 add_executable(hybrid_of …) 了，就只把檔案補上去
    target_sources(hybrid_of PRIVATE ${APP_SOURCES})
  endif()

  target_include_directories(hybrid_of PRIVATE src)
  target_link_libraries(hybrid_of PRIVATE hybrid_of_core)
  if(USE_COINOR)
    target_link_libraries(hybrid_of PRIVATE milp_te)
  endif()

  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(hybrid_of PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  if (WIN32)
    target_link_libraries(hybrid_of PRIVATE ws2_32)
  endif()
endif()



