#ifndef PROJECTDOCUMENT_H
#define PROJECTDOCUMENT_H

#include <QString>
#include <QDomDocument>

namespace posegui {

class ProjectDocument
{
public:
    ProjectDocument();
    ~ProjectDocument();
public:
    void read( const QString &filename );
    void write( const QString &filename );
    void setDocument( const QDomDocument &document );
    const QDomDocument& getDocument() const;
private:
    void validate();
private:
    QDomDocument document;
private:
    static const QString VALIDATOR_FILEPATH;
};

}

#endif // PROJECTDOCUMENT_H
