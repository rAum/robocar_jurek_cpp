file(GLOB LIB_CLIPS_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/libCLIPS/*.c
)

set(CMAKE_CXX_FLAGS "-std=c99 -Wall -Wundef -Wpointer-arith -Wshadow -Wcast-qual -Wcast-align -Winline -Wmissing-declarations -Wredundant-decls -Wmissing-prototypes -Wnested-externs -Wstrict-prototypes -Waggregate-return -Wno-implicit $<")

set(CMAKE_EXE_LINKER_FLAGS "-lm")

add_library(libraryCLIPS STATIC ${LIB_CLIPS_SRCS})

add_executable(main main.c)

target_link_libraries(main libraryCLIPS)

