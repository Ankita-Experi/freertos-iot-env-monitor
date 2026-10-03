// Jenkins declarative pipeline for the FreeRTOS environmental monitor.
//
//   1. Fetch the vendor libraries (git submodules)
//   2. Unit tests, in parallel:
//        - firmware logic in C (BME280 math, wire protocol, ISO-TP, UDS) with ASan/UBSan
//        - Python tools, including the UDS tester run end-to-end against the C server
//   3. Cross-compile both firmware variants for the STM32F446RE, in parallel
//   4. Archive the .elf / .bin / .hex / .map images
//
// The agent needs arm-none-eabi-gcc, CMake, make, a host C compiler and
// python3-pytest. ci/jenkins/Dockerfile builds a Jenkins image that has all of
// them -- see docs/jenkins.md.

pipeline {
    agent any

    environment {
        TOOLCHAIN = 'cmake/arm-none-eabi.cmake'
    }

    options {
        timestamps()
        timeout(time: 30, unit: 'MINUTES')
        buildDiscarder(logRotator(numToKeepStr: '20'))
        disableConcurrentBuilds()
    }

    triggers {
        // Check GitHub for new commits every ~15 minutes. Replace with a
        // GitHub webhook when Jenkins is reachable from the internet.
        pollSCM('H/15 * * * *')
    }

    stages {
        stage('Checkout submodules') {
            steps {
                sh 'git submodule update --init --depth 1'
            }
        }

        stage('Toolchain') {
            steps {
                sh '''
                    arm-none-eabi-gcc --version | head -n 1
                    cmake --version | head -n 1
                    python3 --version
                '''
            }
        }

        stage('Unit tests') {
            parallel {
                stage('Firmware logic (C)') {
                    steps {
                        sh 'make -C firmware/stm32/test test_host test_diag'
                        sh 'cd firmware/stm32/test && ./test_host && ./test_diag'
                    }
                }
                stage('Tools + UDS end-to-end (pytest)') {
                    steps {
                        sh 'python3 -m pytest tools -q --junitxml=build/test-results/pytest.xml'
                    }
                    post {
                        always {
                            junit allowEmptyResults: true, testResults: 'build/test-results/*.xml'
                        }
                    }
                }
            }
        }

        stage('Firmware build') {
            parallel {
                stage('Debug') {
                    steps {
                        dir('firmware/stm32') {
                            sh 'cmake -B build-debug -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DCMAKE_BUILD_TYPE=Debug'
                            sh 'cmake --build build-debug -j "$(nproc)"'
                        }
                    }
                }
                stage('Release, CAN self-test') {
                    steps {
                        dir('firmware/stm32') {
                            sh '''
                                cmake -B build-selftest -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
                                      -DCMAKE_BUILD_TYPE=Release -DCAN_SELF_TEST=ON
                            '''
                            sh 'cmake --build build-selftest -j "$(nproc)"'
                        }
                    }
                }
            }
        }
    }

    post {
        success {
            // env_monitor.elf / .bin / .hex / .map from both builds
            archiveArtifacts artifacts: 'firmware/stm32/build-*/env_monitor.*', fingerprint: true
        }
        failure {
            echo 'Build failed: open the failing stage in Stage View / Blue Ocean for its log.'
        }
    }
}
