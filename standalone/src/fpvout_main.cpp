#include "fpvout.h"

int main() {
    int width = 1280;
    int height = 720;
    FPVout fpvout(width, height);
    return fpvout.run();
}