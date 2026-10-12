cmake_minimum_required(VERSION 3.10)
project(mpc_cpp_sim)

# Impostiamo C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Verifica la variabile d'ambiente Acados
if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(FATAL_ERROR "ACADOS_SOURCE_DIR non è definita! Esegui: export ACADOS_SOURCE_DIR=/percorso/a/acados")
endif()
set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})

include_directories(
    include
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include
    c_generated_code
)

link_directories(${ACADOS_DIR}/lib)

# 1. Trova tutti i file .c dentro c_generated_code (solver, modello, costi, vincoli)
file(GLOB_RECURSE ACADOS_C_FILES "c_generated_code/*.c")

# 2. Acados genera anche un file main_<tuo_modello>.c di esempio. 
# Dobbiamo escluderlo per non avere conflitti con il nostro main.cpp
list(FILTER ACADOS_C_FILES EXCLUDE REGEX "main_.*\\.c")

# 3. Creiamo l'eseguibile unendo il nostro C++ e il C generato da Acados
add_executable(mpc_sim
    src/main.cpp
    src/gait_planner.cpp
    ${ACADOS_C_FILES}
)

# 4. Linkiamo le dipendenze esterne (solo acados base, hpipm e blasfeo)
target_link_libraries(mpc_sim
    acados
    hpipm
    blasfeo
    m
)