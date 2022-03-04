
#ifndef BEAUTIFUL_BULLET_GRAPHICS_ABSTRACTGRAPHICS2_HPP
#define BEAUTIFUL_BULLET_GRAPHICS_ABSTRACTGRAPHICS2_HPP

#include <cstddef>

namespace beautiful_bullet {
    class Simulator2;

    namespace graphics {
        class AbstractGraphics2 {
        public:
            AbstractGraphics2()
                : _desiredFPS(40), _frameCounter(0), _init(false), _done(true), _pause(false) {}

            bool done() { return _done; }

            bool pause() { return _pause; }

            size_t desiredFPS() { return _desiredFPS; }

            AbstractGraphics2& setDesiredFPS(size_t desiredFPS)
            {
                _desiredFPS = desiredFPS;
                return *this;
            }

            virtual bool init(Simulator2& simulator) { return true; }

            virtual bool refresh() { return true; }

        protected:
            bool _init, _done, _pause;

            size_t _frameCounter, _renderPeriod, _desiredFPS;
        };
    } // namespace graphics
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_GRAPHICS_ABSTRACTGRAPHICS2_HPP