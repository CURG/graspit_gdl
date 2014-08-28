#ifndef WORLDCONTROLLER_H
#define WORLDCONTROLLER_H

#include <QObject>

class GraspableBody;

namespace bci_experiment
{

    class WorldController: public QObject
    {
        Q_OBJECT
    public:

        static WorldController * getInstance();


        // Highlighting functionality should move to the view controller
        // The method here should only care about setting the next target
        // and possibly emitting a signal that the target has been emitted.
        void highlightAllBodies();
        void unhighlightAllBodies();
        void highlightCurrentBody(GraspableBody * currentTarget);

    private:
        static WorldController * worldController;

        WorldController(QObject *parent = 0);
    };
}

#endif // WORLDCONTROLLER_H
