FROM ubuntu:24.04 AS builder

ARG CMAKE_BUILD_PARALLEL_LEVEL=2

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    libeigen3-dev \
    libgtest-dev \
    pybind11-dev \
    python3 \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && cmake --build build --parallel "${CMAKE_BUILD_PARALLEL_LEVEL}" \
 && cmake --install build --prefix /opt/libgnsspp \
 && py_site="$(find /opt/libgnsspp/lib -type d -path '*/site-packages' | head -n 1)" \
 && test -n "${py_site}" \
 && mkdir -p /opt/libgnsspp/lib/python3 \
 && ln -sf "${py_site}" /opt/libgnsspp/lib/python3/site-packages

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    python3 \
    python3-matplotlib \
    python3-numpy \
 && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/libgnsspp /opt/libgnsspp

ENV PATH=/opt/libgnsspp/bin:${PATH}
ENV PYTHONPATH=/opt/libgnsspp/lib/python3/site-packages

COPY --from=builder /src/configs /opt/libgnsspp/configs

WORKDIR /workspace
EXPOSE 8085

# Smoke-test: CLI and Python binding must both load successfully.
RUN gnss --help >/dev/null \
 && gnss ppp --help >/dev/null \
 && python3 -c "import libgnsspp" >/dev/null

# Default entrypoint: run the unified gnss CLI.
# One-liner examples (mount data with -v):
#
#   docker run --rm -v "$PWD/data:/data" -v "$PWD/out:/out" libgnsspp \
#     ppp --config /opt/libgnsspp/configs/examples/clas_kinematic.toml \
#         --obs /data/rover.obs --nav /data/base.nav \
#         --ssr /data/ssr_expanded.csv \
#         --out /out/solution.pos --geojson /out/solution.geojson
#
#   docker run --rm libgnsspp ppp --help
ENTRYPOINT ["gnss"]
CMD ["--help"]
