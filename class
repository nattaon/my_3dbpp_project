void PCLViewer::StartTimerEvent()
{
	StopTimerEvent();//cancel previous timer (if exist)

    //nframes = 0; // init
    timerId = startTimer(100); //call timerEvemt every 100 msec
    time.start(); // start time
	cout << "start new timer id = " << timerId << endl;
}
void PCLViewer::StopTimerEvent()
{
	//timerId will = 0, if no timerEvent set
	cout << "stop timer id = " << timerId << endl;
	if (timerId!=0)
	{
		killTimer(timerId);
	}
	timerId = 0;	
}