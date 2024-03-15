FROM osrf/ros:noetic-desktop

ENV LANG='en_US.UTF-8'
ENV LANGUAGE='en_US:en'
ENV LC_ALL='en_US.UTF-8'
ENV USER=root
WORKDIR /workspace

COPY init-apt.sh /root/init-apt.sh
RUN /bin/bash /root/init-apt.sh

COPY nix.conf /tmp/nix.conf
COPY flake.nix /tmp/flake.nix
COPY init-nix.sh /root/init-nix.sh
RUN /bin/bash /root/init-nix.sh
ENV NIX_PATH=nixpkgs=/root/.nix-defexpr/channels/nixpkgs:/root/.nix-defexpr/channels
ENV NIX_SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

COPY init-cleanup.sh /root/init-cleanup.sh
RUN /bin/bash /root/init-cleanup.sh
