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
    void loadProjectEvent();
    void closeProjectEvent();
    void createProjectEvent();

protected:
    void resizeEvent( QResizeEvent* event ) override;
    void keyPressEvent(QKeyEvent *event) override;
private:
    const int SCALE_FACTOR = 80;
};

#endif // FRAMETABLEWIDGET_H
