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
RUN apt -y install sudo
RUN apt -y install python3-pip libz-dev wget vim less
RUN apt -y install python3.7 python3-venv python3.7-venv python3.7-dev
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1
RUN pip3 install --upgrade pip
RUN pip3 install numpy ipython

# add useful commands to .bashrc
RUN echo 'alias kk="kill -9 %"' >> ~/.bashrc

# add ssh server
RUN apt-get install -y openssh-server
RUN mkdir /var/run/sshd
RUN echo 'root:root123' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

EXPOSE 22

CMD ["/usr/sbin/sshd", "-D"]
