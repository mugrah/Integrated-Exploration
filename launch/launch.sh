   rm ../maps/*png
   rm ../maps/*txt
   rm ../maps/*yaml

   program1='roslaunch pioneer3at explorer.launch'
   $program1
   sleep 10
   program2='roslaunch pioneer3at robot.launch robot:=robot1'
