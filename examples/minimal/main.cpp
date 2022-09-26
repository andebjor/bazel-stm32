int main() {
    int x = 0;
    int y = 3;

    for (;;) {
        volatile int z = x + y;
    }

    return 0;
}

