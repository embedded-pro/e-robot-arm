FROM gabrielfrasantos/embedded-devcontainer-cpp:v7.5.1@sha256:58615703161844d1b53db6342f9c962fac0a29f46bb0d457dd1a459811a97644

COPY install-qt-quick3d.sh /tmp/install-qt-quick3d.sh
RUN /tmp/install-qt-quick3d.sh && rm -rf /var/lib/apt/lists/* /tmp/install-qt-quick3d.sh

HEALTHCHECK NONE
