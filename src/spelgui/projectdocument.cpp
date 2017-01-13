// SPEL definitions
#include "predef.hpp"

#include "projectdocument.h"
#include "exceptions.h"

#include <QFile>
#include <QTextStream>
#include <QXmlSchema>
#include <QXmlSchemaValidator>
#include "xmlmessagehandler.h"

namespace posegui {

  //PUBLIC

  ProjectDocument::ProjectDocument()
    : document()
  {}

  ProjectDocument::~ProjectDocument()
  {}

  void ProjectDocument::read(const QString &filename){
    QFile file(filename);
    if (file.open(QFile::ReadOnly)){
      document.setContent(&file);
    }
    else{
      QString msg = file.errorString();
      file.close();
      throw FileNotOpen(msg);
    }
    file.close();
    validate();
  }

  void ProjectDocument::write(const QString &filename){
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)){
      QTextStream out(&file);
      document.save(out, 0);
    }
    else{
      QString msg = file.errorString();
      file.close();
      throw FileNotOpen(msg);
    }
    file.close();
  }

  void ProjectDocument::setDocument(const QDomDocument &document){
    this->document = document;
  }

  const QDomDocument& ProjectDocument::getDocument() const{
    return document;
  }


  //PRIVATE

  const QString ProjectDocument::VALIDATOR_FILEPATH = ":/root/resources/xml/project.xsd";

  void ProjectDocument::validate(){
    QFile file(VALIDATOR_FILEPATH);
    QXmlSchema schema;
    XmlMessageHandler messagesHandler;
    schema.setMessageHandler(&messagesHandler);
    if (file.open(QFile::ReadOnly)){
      schema.load(&file);
    }
    else{
      QString msg = file.errorString();
      file.close();
      throw FileNotOpen(msg);
    }
    file.close();
    //check for errors
    bool errorOccured = false;
    if (!schema.isValid()){
      errorOccured = true;
    }
    else{
      QXmlSchemaValidator validator(schema);
      if (!validator.validate(document.toByteArray())){
        errorOccured = true;
      }
    }
    if (errorOccured){
      QString msg = messagesHandler.errorMessage();
      throw InvalidProjectStructure(msg);
    }
  }

}

