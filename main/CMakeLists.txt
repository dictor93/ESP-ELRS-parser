idf_component_register(SRCS "esp-elrs-rx.cpp" "./src/Controller/Controller.cpp"  "./src/UartELRSParser/UartELRSParser.cpp"  "./src/PWMMotorsControl/PWMMotorsControl.cpp"
                    INCLUDE_DIRS "./src/Controller" "./src/PWMMotorsControl/" "./src/UartELRSParser/")

