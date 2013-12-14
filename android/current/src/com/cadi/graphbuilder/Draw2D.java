package com.cadi.graphbuilder;

// import com.cadi.monitor.R;

import android.view.View;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;
import android.util.AttributeSet;
import java.util.ArrayList;

public class Draw2D extends View{
	    private Paint paintBrushBlue = null;
	    private Paint paintBrushGreen = null;
	    private Paint paintBrushRed = null;
	    private Paint paintBrushBlack = null;
	    
	    private Paint paintBrushRh = null;
	    private Paint paintBrushTemp = null;
	    private Paint paintBrushEc = null;
	    private Paint paintBrushPh = null;
	    // private int[] tempBuffer;
	    private int bufferPointer = 0;
	    ArrayList<Integer> al;	// humidity
	    ArrayList<Integer> phList;	// humidity
	    ArrayList<Integer> ecList;	// humidity
	    ArrayList<Integer> tempList;	// humidity
	    
	    
	    public void onCreate(){
//	    	List<Point> points = new ArrayList<Point>();
	    	
	    }
	    
	    public Draw2D(Context context)
	    {
	       super(context);
	    }
	    
	    public Draw2D(Context context, AttributeSet attrs)
	    {
	       super(context, attrs);	// musthave to run the canvas properly


	    	// tempBuffer = new int[200];
	    	al = new ArrayList<Integer>();	// define buffers for incoming data
	    	phList = new ArrayList<Integer>();
	    	ecList = new ArrayList<Integer>();
	    	tempList = new ArrayList<Integer>();
	       
/*	       paintBrushBlue = new Paint();
	       paintBrushBlue.setColor(Color.BLUE);
	       
	       paintBrushGreen = new Paint();
	       paintBrushGreen.setColor(Color.GREEN);
	       
	       paintBrushRed = new Paint();
	       paintBrushRed.setColor(Color.RED); */

	       paintBrushBlack = new Paint();		// setting color with name
	       paintBrushBlack.setColor(Color.BLACK);
	       
	       
	       paintBrushTemp = new Paint();	// setting custom colors with numbers ,for graphs' fill
	       paintBrushTemp.setARGB(200, 250, 250, 0);
	       
	       paintBrushRh = new Paint();
	       paintBrushRh.setARGB(200, 0, 10, 150);
	       
	       paintBrushEc = new Paint();
	       paintBrushEc.setARGB(200, 120, 200, 10);
	       
	       paintBrushPh = new Paint();
	       paintBrushPh.setARGB(200, 150, 0, 10);
	       
	       
	    }

	    public void onDraw(Canvas canvas) 
	    {
	    	
	    	int g1posx = 0;	// left top point of graph nr 1
	    	int g1posy = 0;	
	    	
	    	int g2posx = 130;	// graph 2
	    	int g2posy = 0;
	    	
	    	int g3posx = 0;
	    	int g3posy = 75;
	    	
	    	int g4posx = 130;
	    	int g4posy = 75;
	    	
	       canvas.drawColor(Color.WHITE);	// set background color
	       canvas.drawRect(g1posx+7,g1posy,g1posx+8,g1posy+50, paintBrushBlack);	// graph 1 coordinate axis
	       canvas.drawRect(g1posx,g1posy+50,g1posx+120,g1posy+51, paintBrushBlack);
	       
	       canvas.drawRect(g2posx+7,g2posy,g2posx+8,g2posy+50, paintBrushBlack);	// graph 2
	       canvas.drawRect(g2posx,g2posy+50,g2posx+120,g2posy+51, paintBrushBlack);
	       
	       canvas.drawRect(g3posx+7,g3posy,g3posx+8,g3posy+50, paintBrushBlack);	// 
	       canvas.drawRect(g3posx,g3posy+50,g3posx+120,g3posy+51, paintBrushBlack);
	       
	       canvas.drawRect(g4posx+7,g4posy,g4posx+8,g4posy+50, paintBrushBlack);	// 
	       canvas.drawRect(g4posx,g4posy+50,g4posx+120,g4posy+51, paintBrushBlack);
	       	          
	       if (al.size()>0) {	// everywhere we need these checks for not emptiness
		  
		       if (al.get(0)>0) {	// even here
			       for (int i=0; i<al.size();i++) {
			    	   canvas.drawRect((g1posx+8+i),g1posy+50,(g1posx+9+i),((g1posy+50-(al.get(i))/2)), paintBrushRh); // draw graph 1 values
			    	   canvas.drawRect((g2posx+8+i),g2posy+50,(g2posx+9+i),((g2posy+50-(tempList.get(i))/2)), paintBrushTemp);
			    	   canvas.drawRect((g3posx+8+i),g3posy+50,(g3posx+9+i),((g3posy+50-(ecList.get(i))/2)), paintBrushEc);
			    	   canvas.drawRect((g4posx+8+i),g4posy+50,(g4posx+9+i),((g4posy+50-(phList.get(i))/2)), paintBrushPh);
			       }
		       }
	       }
	    }
	    
	    // receives the data for graphs
	    public void pushValues(int tempValue, int rhValue, int phValue, int ecValue) {
	    	
	    	int GraphHistoryLength = 100;	// number of buffer to display / shirina grafika po x
	    	if (al.size()>GraphHistoryLength){ // if number of elements in buffer more than with of graph, start shifting buffers
	    		for (int i=0; i<GraphHistoryLength;i++) {
	    			al.set(i, al.get(i+1));
	    			tempList.set(i, tempList.get(i+1));
	    			phList.set(i, phList.get(i+1));
	    			ecList.set(i, ecList.get(i+1));
	    		}
	    		this.al.set(GraphHistoryLength, rhValue);	// write the last received data to the end of buffers
	    		this.tempList.set(GraphHistoryLength, tempValue);
	    		this.phList.set(GraphHistoryLength, phValue);
	    		this.ecList.set(GraphHistoryLength, ecValue);
	    	}
	    	else{
	    		this.al.add(rhValue);	// otherwise just fill the buffer
	    		ecList.add(ecValue);
	    		phList.add(phValue);
	    		tempList.add(tempValue);
	    	}
	    	invalidate();	// redraw graphs
	    }
	 
}

