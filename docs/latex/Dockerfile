FROM debian:bookworm-slim
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    texlive-xetex texlive-lang-chinese texlive-latex-extra texlive-fonts-recommended \
    fonts-noto-cjk fonts-texgyre fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
CMD ["sh", "docs/latex/build.sh"]
