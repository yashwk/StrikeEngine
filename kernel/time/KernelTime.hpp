#pragma once

namespace StrikeEngine::Kernel {

    class KernelTime {
    public:
        KernelTime() : time_s(0.0) {}

        void reset() {
            time_s = 0.0;
        }

        void advance(double dt) {
            time_s += dt;
        }

        double currentTime() const {
            return time_s;
        }

    private:
        double time_s;
    };

} // namespace StrikeEngine::Kernel
