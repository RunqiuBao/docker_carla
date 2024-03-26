FROM carlasim/carla:0.9.15

# Docker実行してシェルに入ったときの初期ディレクトリ（ワークディレクトリ）の設定
WORKDIR /root/

# nvidia-container-runtime（描画するための環境変数の設定）
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install necessary packages
USER root
RUN apt update || apt -y install python3-pip libz-dev

# add useful commands to .bashrc
RUN echo 'alias kk="kill -9 %"' >> ~/.bashrc
