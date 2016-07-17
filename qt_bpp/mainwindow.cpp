#include "mainwindow.h"
#include "build/ui_mainwindow.h"
#include <QMessageBox>
#include <iostream>
/*
REMAIN ISSUE
-some case ording is not in order
-light when rotate to back
-color item in tree list+sort them by order?
-memory management (free pointer?)
-load and append funtion?
*/

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	last_select_item_index = 0;

	total_boxes = 0;

	ui->treeWidget->header()->show();
	ui->treeWidget->setEditTriggers(QAbstractItemView::DoubleClicked);

	//some initial 
	//bin
	//ui->in_bin_w->setText(QString("250"));
	//ui->in_bin_h->setText(QString("200"));
	//ui->in_bin_d->setText(QString("250"));
	//box
	AddNewIntItemToList(200, 100, 50);
	AddNewIntItemToList(200, 50, 100);
	AddNewIntItemToList(100, 200, 50);
	AddNewIntItemToList(100, 50, 200);
	AddNewIntItemToList(50, 100, 200);
	AddNewIntItemToList(50, 200, 100);

	/////////////////
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(PressedTreeItem(QTreeWidgetItem *, int)));


	connect(ui->bt_add_box, SIGNAL(pressed()), this, SLOT(PressedAddBox()));
	connect(ui->bt_remove, SIGNAL(pressed()), this, SLOT(PressedRemoveBox()));
	connect(ui->bt_load, SIGNAL(pressed()), this, SLOT(PressedLoadDb()));
	connect(ui->bt_save, SIGNAL(pressed()), this, SLOT(PressedSaveDb()));
	connect(ui->bt_clearall, SIGNAL(pressed()), this, SLOT(PressedClearAll()));

	connect(ui->bt_binpacking, SIGNAL(pressed()), this, SLOT(PressedBinPacking()));
    connect(ui->bt_reset, SIGNAL(pressed()), this, SLOT(PressedReset()));
	connect(ui->bt_showorder, SIGNAL(pressed()), this, SLOT(PressedShowOrder()));
	connect(ui->bt_previous_order, SIGNAL(pressed()), this, SLOT(PressedPreviousOrder()));
	connect(ui->bt_next_order, SIGNAL(pressed()), this, SLOT(PressedNextOrder()));



    ui->widget->SetInitialBin( GetBinWidth(), GetBinHeight(), GetBinDepth());

    binpack = new CalculateBPP();
	txtfile = new ReadWriteFile();

}
void MainWindow::AddNewIntItemToList(int item1,int item2,int item3)
{
	
	QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
	total_boxes++;

    item->setText(0, QString::number(total_boxes));
	item->setText(1, QString::number(item1));
	item->setText(2, QString::number(item2));
	item->setText(3, QString::number(item3));

	item->setTextAlignment(0, Qt::AlignHCenter);
	item->setTextAlignment(1, Qt::AlignHCenter);
	item->setTextAlignment(2, Qt::AlignHCenter);
	item->setTextAlignment(3, Qt::AlignHCenter);

	item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
	

	ui->treeWidget->addTopLevelItem(item);

}
void MainWindow::AddNewStringItemToList(QString item1, QString item2, QString item3)
{

	QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
	total_boxes++;

	item->setText(0, QString::number(total_boxes));
	item->setText(1, item1);
	item->setText(2, item2);
	item->setText(3, item3);

	item->setTextAlignment(0, Qt::AlignHCenter);
	item->setTextAlignment(1, Qt::AlignHCenter);
	item->setTextAlignment(2, Qt::AlignHCenter);
	item->setTextAlignment(3, Qt::AlignHCenter);

	item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);

	ui->treeWidget->addTopLevelItem(item);

}
void MainWindow::GetBoxesSize()
{
	int *boxes_width = new int[total_boxes];
	int *boxes_height = new int[total_boxes];
	int *boxes_depth = new int[total_boxes];

	for (int i = 0; i < total_boxes; ++i)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);

		boxes_width[i] = item->text(1).toInt();
		boxes_height[i] = item->text(2).toInt();
		boxes_depth[i] = item->text(3).toInt();

	}

	boxes_w = boxes_width;
	boxes_h = boxes_height;
	boxes_d = boxes_depth;

	//delete[] boxes_width;  // if delete error occur
	//delete[] boxes_height;
	//delete[] boxes_depth;


}
void MainWindow::PressedTreeItem(QTreeWidgetItem *item, int col)
{

	
	//std::cout << "mousePressEvent " << ui->treeWidget->currentIndex().row() << std::endl;

	QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);
	last_selected_item->setBackgroundColor(0, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(1, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(2, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(3, QColor(255, 255, 255));


	item->setBackgroundColor(0, QColor(200, 200, 200));
	item->setBackgroundColor(1, QColor(200, 200, 200));
	item->setBackgroundColor(2, QColor(200, 200, 200));
	item->setBackgroundColor(3, QColor(200, 200, 200));
	last_select_item_index = ui->treeWidget->currentIndex().row();

}

void MainWindow::PressedAddBox()
{
    QString txt_width = ui->in_box_w->text();
	QString txt_height = ui->in_box_h->text();
	QString txt_depth = ui->in_box_d->text();

	AddNewStringItemToList(txt_width, txt_height, txt_depth);
    
	//width.toDouble();
    //item->setText(0,QString("a"));
    //item->setBackgroundColor(0,QColor(185, 20, 120));

}
void MainWindow::PressedRemoveBox()
{
    QTreeWidgetItem* selected_item = ui->treeWidget->currentItem();
    delete selected_item;

    total_boxes--;

    //rerun index number
	for (int i = 0; i < total_boxes; ++i)
    {
        QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
        item->setText(0, QString::number(i + 1));

    }
}
void MainWindow::PressedLoadDb()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Image"), "../", tr("Iext Files (*.txt)"));

	std::cout << "Got filename: " << fileName.toStdString() << std::endl;
	
	
	
	txtfile->OpenTxtFileBoxes(fileName.toStdString());

	int total_boxes_from_txt = txtfile->GetTotalBox();

	int bin_w_from_txt = txtfile->GetBinWidth();
	int bin_h_from_txt = txtfile->GetBinHeight();
	int bin_d_from_txt = txtfile->GetBinDepth();

	int *boxes_w_from_txt = txtfile->GetBoxesWidth();
	int *boxes_h_from_txt = txtfile->GetBoxesHeight();
	int *boxes_d_from_txt = txtfile->GetBoxesDepth();	
	std::cout << "Got total_boxes: " << total_boxes_from_txt << std::endl;
	
	PressedClearAll();

	ui->in_bin_w->setText(QString::number(bin_w_from_txt));
	ui->in_bin_h->setText(QString::number(bin_h_from_txt));
	ui->in_bin_d->setText(QString::number(bin_d_from_txt));



	for (int i = 0; i < total_boxes_from_txt; i++)
	{
		AddNewIntItemToList(boxes_w_from_txt[i], boxes_h_from_txt[i], boxes_d_from_txt[i]);
		//AddNewIntItemToList(boxes_w[i], boxes_h[i], boxes_d[i]);
	}
	




}
void MainWindow::PressedSaveDb()
{
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Open Image"), "../", tr("Iext Files (*.txt)"));

	std::cout << "Set filename: " << fileName.toStdString() << std::endl;

	GetBoxesSize();

	txtfile->SaveTxtFileBoxes(fileName.toStdString(), GetTotalBox(),
		GetBinWidth(), GetBinHeight(), GetBinDepth(),
		boxes_w, boxes_h, boxes_d);
}
void MainWindow::PressedClearAll()
{
	for (int i = total_boxes-1; i >=0 ; i--)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		delete item;
	}
	total_boxes = 0;
}

void MainWindow::PressedBinPacking()
{
	//0.set packing type
	//std::cout << "packtype " << ui->comboBox_packMethod->currentIndex() << std::endl;
	binpack->SetPackingType(ui->comboBox_packMethod->currentIndex());

    //1.set bin size
    int bin_width = GetBinWidth();
    int bin_height = GetBinHeight();
    int bin_depth = GetBinDepth();

    binpack->SetBinSize(bin_width, bin_height, bin_depth);


	//2.set boxes size
	GetBoxesSize();
	binpack->SetBoxesSize(total_boxes, boxes_w, boxes_h, boxes_d);

		//3.init box position
		int *boxes_x_pos = new int[total_boxes];
		int *boxes_y_pos = new int[total_boxes];
		int *boxes_z_pos = new int[total_boxes];
		int *boxes_bin_num = new int[total_boxes];


	if (binpack->CheckAllCanFit())
	{

		for (int i = 0; i < total_boxes; ++i)
		{
			boxes_x_pos[i] = 0;
			boxes_y_pos[i] = 0;
			boxes_z_pos[i] = 0;
			boxes_bin_num[i] = 0;
		}

		//4.calc & get result box position
		binpack->SetLimit(GetNodeLimit(), GetIterLimit(), GetTimeLimit());
		binpack->CalculateBinpack(boxes_x_pos, boxes_y_pos, boxes_z_pos, boxes_bin_num);


		// check if use bin >1
		int item_not_fit = 0;
		for (int i = 0; i < total_boxes; i++)
		{
			/*std::cout
			<< i << ":"
			<< boxes_width[i] << " " << boxes_height[i] << " " << boxes_depth[i] << " "
			<< "bin_num:" << boxes_bin_num[i] << " "
			<< "pos:"
			<< boxes_x_pos[i] << " " << boxes_y_pos[i] << " " << boxes_z_pos[i] << " "
			<< std::endl;
			*/
			if (boxes_bin_num[i] != 1) item_not_fit++;

		}
		if (item_not_fit != 0)
		{
			//std::cout << " cannot fit another " << item_not_fit << " boxes to bin" << std::endl;
			QString text = "Another " + QString::number(item_not_fit) + " boxes cannot fit in to bin"; // CORRECT
			QMessageBox::information(0, QString("Cannot fit all boxes"), text, QMessageBox::Ok);
		}
		//remove remain boxes

		binpack->SortBoxesOrder();

		ui->widget->SetShowBinpacking(
			total_boxes,
			bin_width, bin_height, bin_depth,
			boxes_w, boxes_h, boxes_d,
			boxes_x_pos, boxes_y_pos, boxes_z_pos);

	}
	else
	{
		QMessageBox::information(0, QString("Cannot fit all boxes"), QString("All Boxes volumn is larger than bin"), QMessageBox::Ok);
	}








	//delete[] boxes_x_pos;  //if delete display will reset to nothing
	//delete[] boxes_y_pos;
	//delete[] boxes_z_pos;
	//delete[] boxes_bin_num;
}
void MainWindow::PressedReset()
{
	ui->widget->SetInitialBin(GetBinWidth(), GetBinHeight(), GetBinDepth());
	ui->widget->ResetWindow();

}
void MainWindow::PressedShowOrder()
{
	//show animation?
}
void MainWindow::PressedPreviousOrder()
{
	ui->widget->DecreaseShowNumber();
}
void MainWindow::PressedNextOrder()
{
	ui->widget->IncreaseShowNumber();
}



int MainWindow::GetTotalBox()
{
	total_boxes=ui->treeWidget->topLevelItemCount();
	return total_boxes;
}
int MainWindow::GetBinWidth()
{
    QString txt_width = ui->in_bin_w->text();
    return txt_width.toInt();
}
int MainWindow::GetBinHeight()
{
    QString txt_height = ui->in_bin_h->text();
    return txt_height.toInt();
}
int MainWindow::GetBinDepth()
{
    QString txt_depth = ui->in_bin_d->text();
    return txt_depth.toInt();
}
int MainWindow::GetNodeLimit()
{
    QString txt = ui->in_node_limit->text();
    return txt.toInt();
}
int MainWindow::GetIterLimit()
{
    QString txt = ui->in_iter_limit->text();
    return txt.toInt();
}
int MainWindow::GetTimeLimit()
{
    QString txt = ui->in_time_limit->text();
    return txt.toInt();
}

MainWindow::~MainWindow()
{
    delete ui;
}


