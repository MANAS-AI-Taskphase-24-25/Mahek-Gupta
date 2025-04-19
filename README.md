final task used two u net
first u net to deblur
second u net to schematic segment
deblur(used ssim lloss+l1 loss) the model has five features that is the number of filters
schematic segment take 19 class with different color meaning different objects in a city scape. i used sde attention block with my model to enhance result
