// SPEL definitions
#include "predef.hpp"

#include "xmlmessagehandler.h"

//PUBLIC

XmlMessageHandler::XmlMessageHandler(QObject *parent)
  :QAbstractMessageHandler(parent)
{
}

QString XmlMessageHandler::errorMessage() const{
  QString resultMessage;
  if (messageType == QtMsgType::QtDebugMsg){
    return resultMessage;
  }
  resultMessage += messageDescr;

  return resultMessage;
}

//PROTECTED

void XmlMessageHandler::handleMessage(QtMsgType type, const QString &description,
  const QUrl &identifier, const QSourceLocation &sourceLocation)
{
  messageType = type;
  messageDescr = description;
  errorLocation = sourceLocation;
}



