FROM ubuntu:24.04 AS builder

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
 && cmake --build build --parallel \
 && cmake --install build --prefix /opt/libgnsspp \
 && py_site="$(find /opt/libgnsspp/lib -type d -path '*/site-packages' | head -n 1)" \
 && if [ -n "${py_site}" ]; then \
      mkdir -p /opt/libgnsspp/lib/python3 \
      && ln -sf "${py_site}" /opt/libgnsspp/lib/python3/site-packages; \
    fi

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

WORKDIR /workspace
EXPOSE 8085

RUN gnss --help >/dev/null \
 && python3 -c "import libgnsspp" >/dev/null 2>&1 || true

ENTRYPOINT ["gnss"]
CMD ["--help"]
