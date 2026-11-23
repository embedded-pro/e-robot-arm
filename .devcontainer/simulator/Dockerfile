FROM gabrielfrasantos/embedded-devcontainer-cpp:v7.4.0@sha256:80d80475b5609f83ddde34f1e251f7b388779b99c7a308943767a2c8228bdc99

COPY install-qt-quick3d.sh /tmp/install-qt-quick3d.sh
RUN /tmp/install-qt-quick3d.sh && rm -rf /var/lib/apt/lists/* /tmp/install-qt-quick3d.sh

HEALTHCHECK NONE
