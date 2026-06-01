FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]

RUN groupadd -g 1000 jaysonbai && \
    useradd -m -u 1000 -g 1000 -s /bin/bash jaysonbai && \
    usermod -aG sudo jaysonbai && \
    echo "jaysonbai ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY .espressif /home/jaysonbai/.espressif/
COPY esp/esp-idf /home/jaysonbai/esp/esp-idf/
COPY esp/IDF_Test /home/jaysonbai/esp/IDF_Test/

RUN chown -R jaysonbai:jaysonbai /home/jaysonbai

ENV IDF_PATH=/home/jaysonbai/esp/esp-idf
ENV IDF_PYTHON_ENV_PATH=/home/jaysonbai/.espressif/python_env/idf5.3_py3.12_env
ENV IDF_TOOLS_PATH=/home/jaysonbai/.espressif
ENV PATH=/home/jaysonbai/.espressif/python_env/idf5.3_py3.12_env/bin:/home/jaysonbai/.espressif/tools/xtensa-esp-elf-gdb/14.2_20240403/xtensa-esp-elf-gdb/bin:/home/jaysonbai/.espressif/tools/riscv32-esp-elf-gdb/14.2_20240403/riscv32-esp-elf-gdb/bin:/home/jaysonbai/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20240530/xtensa-esp-elf/bin:/home/jaysonbai/.espressif/tools/riscv32-esp-elf/esp-13.2.0_20240530/riscv32-esp-elf/bin:/home/jaysonbai/.espressif/tools/esp32ulp-elf/2.38_20240113/esp32ulp-elf/bin:/home/jaysonbai/.espressif/tools/openocd-esp32/v0.12.0-esp32-20241016/openocd-esp32/bin:/home/jaysonbai/esp/esp-idf/tools:/home/jaysonbai/esp/esp-idf/components/espcoredump:/home/jaysonbai/esp/esp-idf/components/partition_table:/home/jaysonbai/esp/esp-idf/components/app_update:$PATH

USER jaysonbai
WORKDIR /home/jaysonbai/esp/IDF_Test
