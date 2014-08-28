#ifndef SEARCH_STATE_UTILS_H_
#define SEARCH_STATE_UTILS_H_
#include "searchState.h"
#include "matvecIO.h"

template <class streamtype>
streamtype & operator<<(streamtype & os, const VariableSet & vs)
{
  // os << vs.getType() << " ";
  //os << vs.getNumVariables();
  for (int i=0; i < vs.getNumVariables(); ++i)
    os << vs.getVariable(i)->getValue() << " ";
  //os << '\n';
  return os;

}

template <class streamtype>
streamtype & operator<<( streamtype & os,  const HandObjectState & vs)
{
  os << *vs.readPosture() << *vs.readPosition();
  return os;

}


template <class streamtype>
streamtype & operator<<( streamtype & os,  const GraspPlanningState & vs)
{
  std::vector<double> dofs(vs.getHand()->getNumDOF(),0);
  vs.readPosture()->getHandDOF(&dofs[0]);
  std::string object_name;
  if(vs.getObject())
  {
    object_name = vs.getObject()->getName().toStdString();
  }
  else{
  object_name = "None";
  }
  os << object_name.c_str() <<' ' <<vs.readPosition()->getCoreTran().rotation().w << ' ' << vs.readPosition()->getCoreTran().rotation().x << ' ' << vs.readPosition()->getCoreTran().rotation().y << ' ' << vs.readPosition()->getCoreTran().rotation().z << ' '
     << vs.readPosition()->getCoreTran().translation().x() << ' ' << vs.readPosition()->getCoreTran().translation().y() << ' ' << vs.readPosition()->getCoreTran().translation().z() << ' ' ;
  for (int i = 0; i < vs.getHand()->getNumDOF(); ++i)
      os << dofs[i] <<" " ;
     //<< *vs.readPosture() 
     os << vs.getEpsilonQuality() <<' ' << vs.getVolume();
  
  const std::vector<double> * qm = vs.getQualityMeasures();
  
  for(std::vector<double>::const_iterator quality_iter = qm->begin(); quality_iter != qm->end(); ++quality_iter )
    os << ' ' << *quality_iter;
  
  
  return os;

}



#endif
