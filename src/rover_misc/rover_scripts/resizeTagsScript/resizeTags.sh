#!/bin/bash

#Make sure to change the templates for the model.sdf to match the change of the resize.

#$numberOfTags=$1
echo "Resize Individuals"
#for ((i = 0; i < 256; i++));do
  #mv ~/models/at$i/materials/textures/atag-$i.png ~/rover_misc_workspace/src/gazebo/models/at$i/materials/textures/atag-$i.png
#done
for ((i = 0; i < 256; i++));do
  echo "Tag number: " $i
  #convert ~/rover_misc_workspace/src/gazebo/models/at$i/materials/textures/atag-$i.png -resize 184x214^! ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atag-$i.png


  #rm ~/rover_misc_workspace/src/gazebo/models/at$i/materials/textures/atag-$i.png
 # mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atag-$i.png ~/rover_misc_workspace/src/gazebo/models/at$i/materials/textures/

  cp ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/individual_template.sdf ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

  perl -pi -e 's/TEMPLATE/at'$i'/g' ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf
  
  mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf ~/rover_misc_workspace/src/gazebo/models/at$i/

done

echo "Resize groups of fours"
for ((i = 0; i < 16; i++));do
    echo "Tag number: atags4_$i"
    #convert ~/rover_misc_workspace/src/gazebo/models/atags4_$i/materials/textures/atags4_$i.png -resize 399x456^!  ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atags4_$i.png

   # rm ~/rover_misc_workspace/src/gazebo/models/atags4_$i/materials/textures/atags4_$i.png
    #mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atags4_$i.png ~/rover_misc_workspace/src/gazebo/models/atags4_$i/materials/textures/

    cp ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/groupOfFour_template.sdf ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

    perl -pi -e 's/TEMPLATE/atags4_'$i'/g' ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

    mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf ~/rover_misc_workspace/src/gazebo/models/atags4_$i/    
done

echo "Resize groups of sixteen"
for ((i = 0; i < 4; i++));do
    echo "Tag number: atags16_$i"
   # convert ~/rover_misc_workspace/src/gazebo/models/atags16_$i/materials/textures/atags16_$i.png -resize 399x456^!  ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atags16_$i.png

   # rm ~/rover_misc_workspace/src/gazebo/models/atags16_$i/materials/textures/atags16_$i.png
    #mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atags16_$i.png ~/rover_misc_workspace/src/gazebo/models/atags16_$i/materials/textures/

    cp ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/groupOfSixteen_template.sdf ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

    perl -pi -e 's/TEMPLATE/atags16_'$i'/g' ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

    mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf ~/rover_misc_workspace/src/gazebo/models/atags16_$i/    
done

for ((i = 0; i < 4; i++));do
    echo "Tag number: atags64_$i"
    #convert ~/rover_misc_workspace/src/gazebo/models/atags16_$i/materials/textures/atags16_$i.png -resize 399x456^!  ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atags16_$i.png

    #rm ~/rover_misc_workspace/src/gazebo/models/atags16_$i/materials/textures/atags16_$i.png
    #mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/atags16_$i.png ~/rover_misc_workspace/src/gazebo/models/atags16_$i/materials/textures/

    cp ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/groupOfSixtyFour_template.sdf ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

    perl -pi -e 's/TEMPLATE/atags64_'$i'/g' ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf

    mv ~/rover_misc_workspace/src/rover_scripts/resizeTagsScript/model.sdf ~/rover_misc_workspace/src/gazebo/models/atags64_$i/    
done

