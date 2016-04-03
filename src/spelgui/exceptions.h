#ifndef EXCEPTIONS
#define EXCEPTIONS

#include <QString>
#include <QException>

//noexcept doesn't supported by MSVC
#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif

namespace posegui {

class FileNotOpen : public QException
{
public:
    FileNotOpen( const char* what_arg )
        : msg(what_arg){}
    FileNotOpen( const QString& what_arg )
        : msg(what_arg.toStdString().c_str()){}
public:
    QException* clone() const{
        return new FileNotOpen(*this);
    }
    void raise() const{
        throw *this;
    }
    const char* what() const NOEXCEPT{
        return msg;
    }
private:
    const char* msg;
};

class FileMissing : public QException
{
public:
    FileMissing( const char* what_arg )
        : msg(what_arg){}
    FileMissing( const QString& what_arg )
        : msg(what_arg.toStdString().c_str()){}
public:
    QException* clone() const{
        return new FileMissing(*this);
    }
    void raise() const{
        throw *this;
    }
    const char* what() const NOEXCEPT{
        return msg;
    }
private:
    const char* msg;
};

class FileNotRead : public QException
{
public:
    FileNotRead( const char* what_arg )
        : msg(what_arg){}
    FileNotRead( const QString& what_arg )
        : msg(what_arg.toStdString().c_str()){}
public:
    QException* clone() const{
        return new FileNotRead(*this);
    }
    void raise() const{
        throw *this;
    }
    const char* what() const NOEXCEPT{
        return msg;
    }
private:
    const char* msg;
};

class InvalidProjectStructure : public QException
{
public:
    InvalidProjectStructure( const char* what_arg )
        : msg(what_arg){}
    InvalidProjectStructure( const QString& what_arg )
        : msg(what_arg.toStdString().c_str()){}
public:
    QException* clone() const{
        return new InvalidProjectStructure(*this);
    }
    void raise() const{
        throw *this;
    }
    const char* what() const NOEXCEPT{
        return msg;
    }
private:
    const char* msg;
};

class InvalidProjectState : public QException
{
public:
    InvalidProjectState( const char* what_arg )
        : msg(what_arg){}
    InvalidProjectState( const QString& what_arg )
        : msg(what_arg.toStdString().c_str()){}
public:
    QException* clone() const{
        return new InvalidProjectState(*this);
    }
    void raise() const{
        throw *this;
    }
    const char* what() const NOEXCEPT{
        return msg;
    }
private:
    const char* msg;
};

}

#endif // EXCEPTIONS

