#ifndef FRAMETABLEWIDGET_H
#define FRAMETABLEWIDGET_H

#include <QTableWidget>

class FrameTableWidget : public QTableWidget
{
    Q_OBJECT
public:
    explicit FrameTableWidget(QWidget *parent = 0);
signals:

public slots:
    void openProjectEvent();
    void closeProjectEvent();
    void createProjectEvent();

protected:
    void resizeEvent( QResizeEvent* event ) override;
};

#endif // FRAMETABLEWIDGET_H
