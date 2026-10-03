# A local Jenkins controller with everything the Jenkinsfile needs.
#
#   docker build -t envmon-jenkins ci/jenkins
#   docker run -d --name envmon-jenkins -p 127.0.0.1:8080:8080 \
#              -v envmon-jenkins-home:/var/jenkins_home envmon-jenkins
#
# Then open http://localhost:8080 -- full walkthrough in docs/jenkins.md.

FROM jenkins/jenkins:lts-jdk17

USER root
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi \
        cmake make gcc libc6-dev \
        python3 python3-pytest \
 && rm -rf /var/lib/apt/lists/*

USER jenkins
COPY --chown=jenkins:jenkins plugins.txt /usr/share/jenkins/ref/plugins.txt
RUN jenkins-plugin-cli --plugin-file /usr/share/jenkins/ref/plugins.txt
