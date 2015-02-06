#ifndef XMLMESSAGEHANDLER_H
#define XMLMESSAGEHANDLER_H

#include <QAbstractMessageHandler>

class XmlMessageHandler : public QAbstractMessageHandler
{
public:
    XmlMessageHandler( QObject * parent = 0 );
public:
    QString errorMessage() const;
protected:
    void handleMessage(QtMsgType type, const QString &description,
                       const QUrl &identifier, const QSourceLocation &sourceLocation) override;
private:
    QtMsgType messageType;
    QString messageDescr;
    QSourceLocation errorLocation;
};

#endif // XMLMESSAGEHANDLER_H
