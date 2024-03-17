idf_component_register(SRCS "esp-elrs-rx.cpp" "esp-elrs-rx.cpp"
                    INCLUDE_DIRS "./src/")
set(COMPONENT_SRCS "./src/Controller/Controller.cpp"
                    "./src/UartELRSParser/UartELRSParser.cpp"
                    "./src/PWMMotorsControl/PWMMotorsControl.cpp"
                   )

