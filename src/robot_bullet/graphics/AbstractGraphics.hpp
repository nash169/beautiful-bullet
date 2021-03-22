
#ifndef ROBOT_BULLET_GRAPHICS_ABSTRACTGRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_ABSTRACTGRAPHICS_HPP

#include <cstddef>

namespace robot_bullet {
    class Simulator;

    namespace graphics {
        class AbstractGraphics {
        public:
            AbstractGraphics(double desiredFPS = 25) : _desired_fps(desiredFPS), _frame_counter(0), _init(false), _done(true), _pause(false) {}

            virtual ~AbstractGraphics() {}

            virtual void init(Simulator& sim)
            {
            }

            virtual bool done() { return _done; }

            virtual bool pause() { return _pause; }

            virtual bool refresh() { return true; }

            virtual void setDesiredFPS(size_t desiredFPS) { _desired_fps = desiredFPS; }

            virtual void setEnable(bool) {}

        protected:
            bool _init, _done, _pause;

            size_t _frame_counter, _render_period, _desired_fps;
        };
    } // namespace graphics
} // namespace robot_bullet

#endif // ROBOTRAISIM_GUI_ABSTRACTGRAPHICS_HPP