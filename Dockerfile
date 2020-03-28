FROM rust

WORKDIR /root/
RUN git clone https://github.com/rgreenblatt/simulation
WORKDIR /root/simulation/
RUN cargo build --release
RUN mkdir bin
RUN cp target/release/simulation bin

