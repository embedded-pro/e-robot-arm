# Pipes SCRIPT into CONSOLE --bus simulated --batch and checks the exit status and the output.
execute_process(
    COMMAND "${CONSOLE}" --bus simulated --batch
    INPUT_FILE "${SCRIPT}"
    OUTPUT_VARIABLE output
    ERROR_VARIABLE errors
    RESULT_VARIABLE status
    TIMEOUT 60
)
message(STATUS "${output}${errors}")
if (NOT status EQUAL EXPECT_STATUS)
    message(FATAL_ERROR "exit status ${status}, expected ${EXPECT_STATUS}")
endif()
string(FIND "${output}" "${EXPECT_TEXT}" found)
if (found EQUAL -1)
    message(FATAL_ERROR "output lacks \"${EXPECT_TEXT}\"")
endif()
