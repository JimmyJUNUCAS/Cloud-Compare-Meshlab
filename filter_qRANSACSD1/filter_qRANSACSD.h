#ifndef QRANSACSDOLUGIN_H
#define QRANSACSDOLUGIN_H

//#include <QObject>
#include <common/interfaces.h>
#include <common/meshmodel.h>


class QRanSacSDPlugin : public QObject, public MeshFilterInterface
{
	Q_OBJECT
	MESHLAB_PLUGIN_IID_EXPORTER(MESH_FILTER_INTERFACE_IID)
	Q_INTERFACES(MeshFilterInterface)

public:
	enum { FP_QRANSACSD  } ;

	QRanSacSDPlugin();

	//virtual QString QRanSacSDPlugin(void) const { return "QRanSacSDPlugin"; }

	virtual QString filterName(FilterIDType filter) const;
	virtual QString filterInfo(FilterIDType filter) const;
	virtual void initParameterSet(QAction *, MeshDocument &/*m*/, RichParameterSet & /*parent*/);
    virtual bool applyFilter(QAction *filter, MeshDocument &md, RichParameterSet & /*parent*/, vcg::CallBackPos * cb) ;
	
	int postCondition(QAction*) const;
	FilterClass getClass(QAction *a);
	FILTER_ARITY filterArity(QAction * filter) const { return FIXED; }
};


#endif