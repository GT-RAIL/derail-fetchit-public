# Creates a MARY TTS docker image that can just be run. Keep in mind the
# licensing requirements for MARY: https://github.com/marytts/marytts/LICENSE.md

FROM openjdk:8

ARG MARYTTS_INSTALLER_ROOT
ARG MARYTTS_VERSION
ARG MARYTTS_VOICE

# Install dependencies
RUN apt-get update \
        && apt-get install -y maven git

# Declare the environment variables after the dependencies so that we can reuse
# caches. Update the PATH as well
ENV MARYTTS_INSTALLER_ROOT ${MARYTTS_INSTALLER_ROOT:-/marytts}
ENV MARYTTS_VERSION ${MARYTTS_VERSION:-5.2}
ENV MARYTTS_VOICE ${MARYTTS_VOICE:-*}
ENV PATH "${MARYTTS_INSTALLER_ROOT}:${PATH}"

# Create the MARY install and install the language files
RUN git clone -b v${MARYTTS_VERSION} https://github.com/marytts/marytts-installer ${MARYTTS_INSTALLER_ROOT} \
    && cd ${MARYTTS_INSTALLER_ROOT} \
    && ./marytts "install:${MARYTTS_VOICE}"

# Change the working dir
WORKDIR ${MARYTTS_INSTALLER_ROOT}

# Set the cmd
EXPOSE 59125
CMD ["marytts", "server"]
