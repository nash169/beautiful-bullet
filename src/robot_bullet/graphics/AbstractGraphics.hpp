
#ifndef ROBOT_BULLET_GRAPHICS_ABSTRACTGRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_ABSTRACTGRAPHICS_HPP

#include <cstddef>

namespace robot_bullet {
    class Simulator;

    namespace graphics {
        class AbstractGraphics {
        public:
            AbstractGraphics()
                : _desiredFPS(40), _frameCounter(0), _init(false), _done(true), _pause(false) {}

            bool done() { return _done; }

            bool pause() { return _pause; }

            size_t desiredFPS() { return _desiredFPS; }

            AbstractGraphics& setDesiredFPS(size_t desiredFPS)
            {
                _desiredFPS = desiredFPS;
                return *this;
            }

            virtual bool init(Simulator& simulator) { return true; }

            virtual bool refresh() { return true; }

        protected:
            bool _init, _done, _pause;

            size_t _frameCounter, _renderPeriod, _desiredFPS;
        };
    } // namespace graphics
} // namespace robot_bullet

#endif // ROBOTRAISIM_GUI_ABSTRACTGRAPHICS_HPP