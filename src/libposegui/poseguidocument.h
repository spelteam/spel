#ifndef POSEGUIDOCUMENT_H
#define POSEGUIDOCUMENT_H

namespace posegui {

class PoseguiDocument
{
public:
    virtual void create() = 0;
    virtual void open() = 0;
    virtual void save() = 0;
    virtual void close() = 0;
};

}
#endif // POSEGUIDOCUMENT_H
