#ifndef PROJECT_H
#define PROJECT_H

#include <QObject>

class Project : public QObject
{
    Q_OBJECT
public:
    explicit Project(QObject *parent = 0);

signals:
    void create();
    void open();
    void close();
    void save();

public slots:

};

#endif // PROJECT_H
