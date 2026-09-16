#include "SerialROS2.hpp"

char** split(char sep, char text[]) {
    char* token = nullptr;
    int len = 0;

    char** tokens = nullptr;
    int n_tokens = 0;

    for (size_t i = 0; i <= strlen(text); i++) {
        if (text[i] != '\n' && text[i] != sep) {
            token = (char*)realloc(token, sizeof(char) * (len + 1));
            token[len] = text[i];
            len++;
        } else {
            if (len > 0) {
                token = (char*)realloc(token, sizeof(char) * (len + 1));
                token[len] = '\0';

                tokens = (char**)realloc(tokens, sizeof(char*) * (n_tokens + 1));
                tokens[n_tokens] = token;
                n_tokens++;

                token = nullptr;
                len = 0;
            }
        }
    }

    tokens = (char**)realloc(tokens, sizeof(char*) * (n_tokens + 1));
    tokens[n_tokens] = nullptr;

    return tokens;
}

void freeSplit(char** t){
    for (int i = 0; t[i] != nullptr; i++) {
        free(t[i]);
    }
    free(t);
}

bool SerialROS2::init(){
    this->pc.set_baud(baud_rate);
    this->pc.set_blocking(false);
    return true;
}

DataRecv SerialROS2::recv(){
    DataRecv data_recv;
    data_recv.num_recv = pc.read(&data_recv.data, MAXIMUM_BUFFER_SIZE);
    return data_recv;
}

void SerialROS2::recvVals(char sep){
    // Accumulate bytes across calls until EOP 'd' arrives.
    static char buf[MAXIMUM_BUFFER_SIZE * 2] = {0};
    static int  buf_len = 0;

    // Append any newly arrived bytes into accumulator
    char tmp[MAXIMUM_BUFFER_SIZE];
    int n = pc.read(tmp, sizeof(tmp));
    for (int i = 0; i < n && buf_len < (int)(sizeof(buf) - 1); i++) {
        buf[buf_len++] = tmp[i];
    }
    buf[buf_len] = '\0';

    // Only process once a complete message (containing EOP 'd') is buffered
    bool complete = false;
    for (int i = 0; i < buf_len; i++) {
        if (buf[i] == 'd') { complete = true; break; }
    }
    if (!complete || this->recvCallback == nullptr) return;

    float* recvs = nullptr;
    int count = 0;
    char** out = split(sep, buf);
    for (int i = 0; out[i] != nullptr; i++) {
        recvs = (float*)realloc(recvs, sizeof(float) * (i + 1));
        recvs[i] = atof(out[i]);
        count++;
    }
    freeSplit(out);
    if (count >= 1) this->recvCallback(recvs);   // was >= 2 — blocked 1-motor mode
    free(recvs);

    // Reset accumulator for next message
    memset(buf, 0, sizeof(buf));
    buf_len = 0;
}

void SerialROS2::send(void* data_send, uint32_t size){
    pc.write(data_send, size);
}

void SerialROS2::attach(void (*func)(), chrono::milliseconds t=2000ms){
    send_timer.attach( callback(func), t);
}