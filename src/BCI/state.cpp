#include "BCI/state.h"
#include <QDebug>

State::State( const QString& name, QState* parent )
    : QState( parent ),
      m_name( name ),
      m_prefix()
{
}

State::State( const QString& name, const QString& prefix, QState* parent )
    : QState( parent ),
      m_name( name ),
      m_prefix( prefix )
{
}

void State::onEntry( QEvent* e )
{
    Q_UNUSED( e );

    // Print out the state we are entering and it's parents
    QString state = m_name;
    State* parent = dynamic_cast<State*>( parentState() );
    while ( parent != 0 )
    {
        state = parent->name() + "->" + state;
        parent = dynamic_cast<State*>( parent->parentState() );
    }
    qDebug() << m_prefix << "Entering state:" << state;
}

void State::onExit( QEvent* e )
{
    Q_UNUSED( e );

    // Print out the state we are exiting and it's parents
    QString state = m_name;
    State* parent = dynamic_cast<State*>( parentState() );
    while ( parent != 0 )
    {
        state = parent->name() + "->" + state;
        parent = dynamic_cast<State*>( parent->parentState() );
    }
    qDebug() << m_prefix << "Exiting state:" << state;
}

void State::addSelfTransition(QObject *sender, const char * signal, const QObject *receiver, const char* slot  )
{
    QSignalTransition * newTransition =
            new QSignalTransition(sender, signal);

    addTransition(newTransition);

    connect(newTransition, SIGNAL(triggered()),
            receiver, slot);
}


