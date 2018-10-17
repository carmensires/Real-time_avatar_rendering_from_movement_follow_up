
package skeletonDetector;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.util.CharsetUtil;
import io.scalecube.socketio.Session;
import io.scalecube.socketio.SocketIOListener;
import io.scalecube.socketio.SocketIOServer;
import javafx.application.Platform;
import javafx.beans.property.ObjectProperty;
import javafx.embed.swing.SwingFXUtils;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

public class SkeletonDetectorController {
	
	@FXML
	private Button cameraButton;
	
	@FXML
	private ImageView originalFrame;
	
	@FXML
	private ImageView morphFrameR;
	
	@FXML
	private ImageView morphFrameG;
	
	@FXML
	private ImageView skeletonFrame;
	
	@FXML
	private Label frase;
	
	@FXML
	private ImageView imagenPostura;
	
	//valores hsv del color rojo de los puntos de la parte derecha del cuerpo que se quieren detectar
	//depende de la luz y el color rojo exacto. Para saber qué valores poner utilizo otro programa (ObjectDetector)
	//private Scalar hsvminR=new Scalar(165.16,146.37,103.46);
	//private Scalar hsvmaxR=new Scalar(180,197.42,162.46);
	private Scalar hsvmaxR = new Scalar(4.35,136.25,180.96);
	private Scalar hsvminR = new Scalar(0,113,136.37);
	
	//valores hsv del color verde de los puntos de la parte izquierda del cuerpo que se quieren detectar
	//también dependen de la luz
	//private Scalar hsvminG=new Scalar(62,80.56,101.41);
	//private Scalar hsvmaxG=new Scalar(88.55,102.83,156.3);
	private Scalar hsvminG=new Scalar(55.16,97,136.37);
	private Scalar hsvmaxG=new Scalar(101.61,144.47,180.96);
	
	// temporizador
	private ScheduledExecutorService timer;
	// el objeto que realiza la captura
	private VideoCapture capture = new VideoCapture();
	//para saber si la cámara esta funcionando o no
	private boolean on; 
	
	//frame anterior y las coordenadas calculadas de los puntos del frame anterior
	Mat prevFrame=new Mat();
	MatOfPoint2f prevCoor=new MatOfPoint2f();
	
	//número de puntos detectados en el frame
	private int nPuntos;
	//para saber si nos encontramos en el momento inicial en el que el usuario tiene una postura determinada con la
	//que hay que calcular los puntos, o si ya hemos pasado ese momento y los estamos calculando con el algoritmo 
	//de seguimiento
	private boolean posturaOk;
	//para ordenar los puntos en un primer instante y utilizarlo luego para dibujar las lineas
	private int[] orden=new int[12];
	private int[] ordenInicial={0,1,2,3,4,5,6,7,8,9,10,11};
	
	Session sesion;
	
	@FXML
	public void startCamera()
	{
		
		//Establecemos las dimensiones de los videos que vamos a mostrar
		this.originalFrame.setFitWidth(350);
		this.originalFrame.setPreserveRatio(true);
		this.morphFrameR.setFitWidth(350);
		this.morphFrameR.setPreserveRatio(true);
		this.morphFrameG.setFitWidth(350);
		this.morphFrameG.setPreserveRatio(true);
		this.skeletonFrame.setFitWidth(700);
		this.skeletonFrame.setPreserveRatio(true);
		this.imagenPostura.setFitWidth(100);
		this.imagenPostura.setPreserveRatio(true);
		
		startSocketIOServer();
		
		//si la cámara no está funcionando hay que ponerla en marcha
		if(!this.on)
		{
			this.capture.open(0);
			//si está capturando algo
			if(capture.isOpened())
			{
				this.on=true;
				this.cameraButton.setText("Stop Camera");
				this.frase.setText("Mantenga la postura que se indica a la derecha para poder reconocer su posición");
				//imagen fija para indicar al usuario como tiene que colocarse ante la cámara
				Image iPostura=getSkeletonImage(); 
				this.imagenPostura.setImage(iPostura);
				
				// capturar un frame cada 33 ms (30 frames/s)
				Runnable frameGrabber = new Runnable() {
					
					@Override
					public void run()
					{
						Image imageToShow = grabFrame();
						originalFrame.setImage(imageToShow);
					}
				};
				
				this.timer=Executors.newSingleThreadScheduledExecutor();
				this.timer.scheduleAtFixedRate(frameGrabber, 0, 33, TimeUnit.MILLISECONDS);
			}
			else
			{
				System.err.println("Error al conectar con la cámara...");
			}
		}
		//si la cámara está funcionando y se pulsa el botón de stop camara, hay que apagarla
		else
		{
			this.on=false;
			this.cameraButton.setText("Start Camera");
			
			try {
				this.timer.shutdown();
				this.timer.awaitTermination(33, TimeUnit.MILLISECONDS);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			this.capture.release();
			
		}
		
		
	}
	
	
	public Image grabFrame()
	{
		//imagen original
		Image imageToShow=null; 
		//frame y coordenadas actuales
		Mat currFrame=new Mat(); 
		MatOfPoint2f currCoor=new MatOfPoint2f();
		//imagen fija para indicar al usuario como tiene que colocarse ante la cámara
		this.imagenPostura.setImage(getSkeletonImage());
		
		if(this.capture.isOpened())
		{
			try{
				
				this.capture.read(currFrame);
				//si no está vacío, se procesa
				if(!currFrame.empty())
				{
					Mat blurredImage=new Mat();
					Mat hsvImage=new Mat();
					Mat maskR =new Mat();
					Mat morphImageR=new Mat();
					Mat maskG=new Mat();
					Mat morphImageG=new Mat();
					
					//crear una pantalla con las mismas características que el frame capturado y ponerla en negro
					Mat skeletonImage=currFrame.clone();
					skeletonImage.setTo(new Scalar(0,0,0));
					
					//para eliminar el ruido de la imagen
					Imgproc.blur(currFrame, blurredImage, new Size(7, 7));
					//convertir a hsv
					Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
					
					//filtramos la imagen indicandole los valores hsv min y max, para que se quede con el color rojo
					Core.inRange(hsvImage, hsvminR, hsvmaxR, maskR);
					//filtramos la imagen indicandole los valores hsv min y max, para que se quede con el color verde
					Core.inRange(hsvImage, hsvminG, hsvmaxG, maskG);
															
					//hacemos dilate y erode para conseguir mayor contraste entre los puntos rojos o verdes y el resto.
					Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24,24));
					Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12,12));
					
					Imgproc.erode(maskR, morphImageR, erodeElement);
					Imgproc.erode(maskR, morphImageR, erodeElement);
					Imgproc.dilate(maskR, morphImageR, dilateElement);
					Imgproc.dilate(maskR, morphImageR, dilateElement);
					
					Imgproc.erode(maskG, morphImageG, erodeElement);
					Imgproc.erode(maskG, morphImageG, erodeElement);
					Imgproc.dilate(maskG, morphImageG, dilateElement);
					Imgproc.dilate(maskG, morphImageG, dilateElement);
	
					//para que se muestre la imagen
					this.onFXThread(this.morphFrameR.imageProperty(), this.Mat2Image(morphImageR));
					this.onFXThread(this.morphFrameG.imageProperty(), this.Mat2Image(morphImageG));
					
					//calculamos las coordenadas de los puntos rojos y verdes, en función de las 
					//máscaras creadas con los filtros
					currCoor=this.findPoints(morphImageR,morphImageG);
					//calculamos el número de puntos
					nPuntos=currCoor.rows();
					//parámetros necesarios para la función de seguimiento de los puntos que indican el estado y si hay error
					MatOfByte status=new MatOfByte();
					MatOfFloat err=new MatOfFloat();
					
					//para ir observando el resultado
					/*System.out.println("");
					System.out.println("");
					System.out.println("Number of points: "+nPuntos);
					System.out.println("PREVCOOR: "+prevCoor.dump());
					System.out.println("C. COOR calculadas directamente: "+currCoor.dump());*/
					
					//dibujar los puntos amarillos en las coordenadas que se capturan directamente en cada frame
					skeletonImage=this.drawPoints(skeletonImage,currCoor, new Scalar(0,255,255),20);
					
					if(nPuntos==12) //se detectan todos los puntos que se tienen que detectar 
					{
						if(posturaOk)//si está bien detectada de antes 
							//solo hay que unir las coordenadas siguiendo el algoritmo de Lucas Kanade
						{
							System.out.println("Postura OK de antes");
							//aplicar el algoritmo de lucas-kanade para sacar las nuevas coordenadas (las que siguen a las anteriores)
							//AQUÍ ES DONDE FALLA, NO CALCULA BIEN LAS NUEVAS COORDENADAS
							Video.calcOpticalFlowPyrLK(prevFrame, currFrame, prevCoor, currCoor, status, err);
							System.out.println("C. COOR lukas canade: "+currCoor.dump());
							//unir las líneas en función de las coordenadas nuevas calculadas con Lucas Kanade
							skeletonImage=drawLines(skeletonImage, currCoor);
							frase.setText("");
							
						}else //en este caso nos dan igual las coordenadas anteriores, no hace falta aplicar
							//Lucas Kanade porque antes no estaba bien colocado. Hay que comprobar si ahora
							//tiene la postura que se indica para poder empezar a seguirle el movimiento
						{
							frase.setText("Mantenga la postura que se indica para poder reconocer su posición");
							//comprobar que están bien colocados (brazos y piernas)
							posturaOk=comprobarCoordenadas(currCoor);
							if(posturaOk) //si están bien colocados dibujamos las líneas, si no nada
							{
								skeletonImage=drawLines(skeletonImage, currCoor);
							}
						}
						
						
					}
					else
					{
						frase.setText("Mantenga la postura que se indica para poder reconocer su posición");
						posturaOk=false;
					}
					
					//dibujar los puntos rojos en las posiciones de las coordenadas calculadas con lucas kanade
					skeletonImage=this.drawPoints(skeletonImage,currCoor,new Scalar(0,0,255),20);
					
					//mostrar la imagen resultante (el esqueleto con lineas y puntos)
					this.onFXThread(this.skeletonFrame.imageProperty(), this.Mat2Image(skeletonImage));
					//la imagen original capturada
					imageToShow=Mat2Image(currFrame);
				}
			}
			catch (Exception e)
			{
				System.err.print("ERROR");
				e.printStackTrace();
			}
				
		}
		//le damos los valores del frame y coordenadas actuales, para usarlos al capturar el siguiente frame
		prevFrame=currFrame.clone();
		prevCoor=currCoor;
		return imageToShow;
	}
		
	
	private boolean comprobarCoordenadas(MatOfPoint2f coor)
	{
		boolean ok=false;
		System.out.println("comprobando coordenadas");
		double[][]c=new double[12][2];
		for(int i=0;i<coor.rows();i++)
			c[i]=coor.get(i, 0);
		//comprobamos que realmente los puntos están colocados como tienen que estar (los pies arriba del todo,
		//después las rodillas... ya que se guarda como si la imagen estuviera invertida verticalmente)
		System.out.println("if "+c[0][1]+" "+c[6][1]+">"+c[1][1]+" "+c[7][1]);
		System.out.println("if "+c[1][1]+" "+c[7][1]+">"+c[2][1]+" "+c[3][1]+" "+c[8][1]+" "+c[9][1]);
		System.out.println("if "+c[2][1]+" "+c[3][1]+" "+c[8][1]+" "+c[9][1]+">"+c[4][1]+" "+c[10][1]);
		System.out.println("if "+c[4][1]+" "+c[10][1]+">"+c[5][1]+" "+c[11][1]);

		if(Math.min(c[0][1], c[6][1])>Math.max(c[1][1], c[7][1]) && Math.min(c[1][1], c[7][1])>Math.max(Math.max(c[2][1], c[3][1]),Math.max(c[8][1],c[9][1])) && Math.min(Math.min(c[2][1], c[3][1]),Math.min(c[8][1],c[9][1]))>Math.max(c[4][1],c[10][1]) && Math.min(c[4][1],c[10][1])>Math.max(c[5][1],c[11][1]))
		{
			ok=true;
			System.out.println("OK.");
			this.orden=this.ordenInicial;

			//ordenarlas porque en esos puntos puede que justo este por encima la cadera o que este por encima la mano
            //ordenar 2 y 3
			if(c[2][0]>c[3][0])
			{
				this.orden[2]=3;
				this.orden[3]=2;
			}
			
			//ordenar 8 y 9
			if(c[8][0]<c[9][0])
			{
				this.orden[8]=9;
				this.orden[9]=8;
			}
		}
		return ok;
	}
	
	//se dibujan las lineas entre las coordenadas, usando el array orden.
	private Mat drawLines(Mat frame, MatOfPoint2f coor)
	{
		Point[] puntos=new Point[12];
		for(int i=0;i<coor.rows();i++)
		{
			double []xy=coor.get(orden[i], 0);
			puntos[i]=new Point(xy[0],xy[1]);
		}
		Imgproc.line(frame,puntos[0] ,puntos[1] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[1] ,puntos[2] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[6] ,puntos[7] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[7] ,puntos[8] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[2] ,puntos[8] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[3] ,puntos[4] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[4] ,puntos[5] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[5] ,puntos[11] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[9] ,puntos[10] , new Scalar(0,255,0),8);
		Imgproc.line(frame,puntos[10] ,puntos[11] , new Scalar(0,255,0),8);
		double p1x=(puntos[2].x+puntos[8].x)/2;
		double p1y=(puntos[2].y+puntos[8].y)/2;
		double p2x=(puntos[5].x+puntos[11].x)/2;
		double p2y=(puntos[5].y+puntos[11].y)/2;
		Imgproc.line(frame,new Point(p1x,p1y) ,new Point(p2x,p2y) , new Scalar(0,255,0),8);
		Imgproc.circle(frame, new Point(p1x,p1y), 20, new Scalar(0,0,255),-1);
		Imgproc.circle(frame, new Point(p2x,p2y), 20, new Scalar(0,0,255),-1);
		return frame;
	}
	

	
	private Image Mat2Image(Mat frame)
	{
		MatOfByte buffer=new MatOfByte();
		Imgcodecs.imencode(".png",frame,buffer);
		return new Image(new ByteArrayInputStream(buffer.toArray()));
	}
	
	//encontrar los puntos rojos y verdes y guardarlos en un array de tipo MatOfPoint2f
	private MatOfPoint2f findPoints(Mat maskedImageR, Mat maskedImageG)
	{
		List<MatOfPoint> contoursR=new ArrayList<>();
		List<MatOfPoint> contoursG=new ArrayList<>();
		Mat hierarchyR = new Mat();
		Mat hierarchyG = new Mat();
		List<Point> coordinates=new ArrayList<Point>();
		MatOfPoint2f coor=new MatOfPoint2f();
		Imgproc.findContours(maskedImageR, contoursR, hierarchyR, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
		Imgproc.findContours(maskedImageG, contoursG, hierarchyG, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
		
		if(hierarchyR.size().height>0 && hierarchyR.size().width>0)
		{
			//creamos un array en el que vamos a almacenar las coordenadas de los puntos del cuerpo
			
			for(int i=0;i>=0;i=(int)hierarchyR.get(0,i)[0])
			{
				Rect rect = Imgproc.boundingRect(contoursR.get(i));
				coordinates.add(new Point(rect.x,rect.y));
			}
			
		}
		
		if(hierarchyG.size().height>0 && hierarchyG.size().width>0)
		{			
			for(int i=0;i>=0;i=(int)hierarchyG.get(0,i)[0])
			{
				Rect rect = Imgproc.boundingRect(contoursG.get(i));
				coordinates.add(new Point(rect.x,rect.y));
			}
			
		}	
		
		coor.fromList(coordinates);
		
		return coor;
	}
	
	//dibujar los puntos en las coordenadas
	private Mat drawPoints(Mat frame, MatOfPoint2f coor, Scalar color, int radio)
	{
		int tam=coor.rows();
		for(int i=0;i<tam;i++)
		{
			double []xy=coor.get(i, 0);
			Imgproc.circle(frame, new Point(xy[0],xy[1]), radio, color,-1);
		}
					
		return frame;
	}
	
	
	private <T> void onFXThread(final ObjectProperty<T> property, final T value)
	{
		Platform.runLater(new Runnable() {
			
			@Override
			public void run()
			{
				property.set(value);
			}
		});
	}
	
	//coger la imagen de un archivo
	private Image getSkeletonImage()
	{
		BufferedImage buff;
		Image i=null;
		try {
			buff = ImageIO.read(new File("/Users/Carmen/Documents/workspace/SkeletonDetector/esqueleto.png"));
			i=SwingFXUtils.toFXImage(buff, null);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return i;
	}
	
	private MatOfPoint2f inventCoordinates()
	{
		List<Point> coordinates=new ArrayList<Point>();
		MatOfPoint2f coor=new MatOfPoint2f();
		
		coordinates.add(new Point(380,554));
		coordinates.add(new Point(354,432));
		coordinates.add(new Point(363,295));
		coordinates.add(new Point(462,305));
		coordinates.add(new Point(423,225));
		coordinates.add(new Point(377,117));
		coordinates.add(new Point(273,540));
		coordinates.add(new Point(272,414));
		coordinates.add(new Point(272,293));
		coordinates.add(new Point(179,320));
		coordinates.add(new Point(217,226));
		coordinates.add(new Point(267,112));
		
		coor.fromList(coordinates);
		return coor;
	}
	
	public void startSocketIOServer (){
		SocketIOServer logServer = SocketIOServer.newInstance(5000 /*port*/);
		logServer.setListener(new SocketIOListener() {
		  public void onConnect(Session session) {
			  sesion=session;
		    System.out.println("Connected: " + session);  
		  }
		  
		  public void onMessage(Session session, ByteBuf message) {
		    System.out.println("Received: " + message.toString(CharsetUtil.UTF_8));
		    message.release();
		    sendCoordinates(inventCoordinates());
		  }
		  
		  public void onDisconnect(Session session) {
		    System.out.println("Disconnected: " + session);  
		  }
		});
		logServer.start();
	}

	public String toJSON(MatOfPoint2f currCoor){
		String[] nombres = {"L_Foot","L_Calf","L_Thigh","L_Hand","L_Forearm","L_UpperArm",
		"R_Foot","R_Calf","R_Thigh","R_Hand","R_Forearm","R_UpperArm"};
		StringBuilder json = new StringBuilder();
		double []xy;
		json.append("[");

		for(int i=0;i<currCoor.rows();i++)
		{
			xy=currCoor.get(ordenInicial[i], 0);
			json.append("{\"name\":\""+nombres[i]+"\",\"x\":"+xy[0]+",\"y\":"+xy[1]+"}");
			if(i<currCoor.rows()-1)
				json.append(",");
		}
		json.append("]");
		return json.toString();
	}
	
	private void sendCoordinates(MatOfPoint2f currCoor)
	{
		String msg=toJSON(currCoor);
		ByteBuf bb = Unpooled.wrappedBuffer(msg.getBytes());
		sesion.send(bb);
	}

}
