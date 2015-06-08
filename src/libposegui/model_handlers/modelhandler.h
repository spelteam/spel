#ifndef MODELHANDLER_H
#define MODELHANDLER_H

namespace posegui {

template<
    typename Model,
    typename Data,
    typename... Args
>
class ModelHandler{
public:
    virtual ~ModelHandler(){}
};

template<
        typename Model,
        typename Data,
        typename Controller
        >
class ModelHandler<Model,Data,Controller>
{
public:
    virtual Model read( const Data& data ) = 0;
    virtual Data write( const Model& model, Controller& controller ) = 0;
    virtual ~ModelHandler(){}
};


}

#endif // MODELHANDLER_H
