#include "App.hpp"

int main(int argc, char** argv)
{
    App app;

    if (!app.init()) {
        return -1;
    }

    app.run();
    app.shutdown();

    return 0;
}
