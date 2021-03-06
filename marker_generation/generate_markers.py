from PIL import Image, ImageDraw, ImageFont
import os


tag_number = 1

number_y = 20
number_size = 100

box_y = number_y + number_size + 20
box_size = (400, 100)
circle_number = 4
circle_size = 70
circle_dist = 100

ar_y = box_y + box_size[1] + 20
ar_size = 340

size = (430, ar_y + ar_size + 20)


img = Image.new('RGBA', size, (255,255,255,255))

# get a font
fnt = ImageFont.truetype('DejaVuSans-Bold.ttf', number_size)
# get a drawing context
d = ImageDraw.Draw(img)

# decimal number
ts = d.textsize(str(tag_number), fnt)[0]
d.text(((img.size[0]-ts)/2, number_y), str(tag_number), font=fnt, fill=(0,0,0,255))

# binary circle counter
d.rectangle(((img.size[0]-box_size[0])/2, box_y, (img.size[0]+box_size[0])/2, box_y + box_size[1]), fill=(0,200,50,255))
for i in range(circle_number):
    if(tag_number & (1 << i)):
        d.ellipse(( img.size[0]/2+((circle_number-1)/2.-i)*circle_dist - circle_size/2, box_y+(box_size[1]-circle_size)/2, 
                    img.size[0]/2+((circle_number-1)/2.-i)*circle_dist + circle_size/2, box_y+(box_size[1]+circle_size)/2), fill=(0,50,200,255))


# ar tag0,
os.system('cd markers; rosrun ar_track_alvar createMarker ' + str(tag_number))

ar_tag = Image.open('markers/MarkerData_' + str(tag_number) + '.png').resize((ar_size,ar_size), Image.NEAREST)
img.paste(ar_tag, ((size[0]-ar_size)/2, ar_y))


#img.show() 
img.save('markers/texture_marker' + str(tag_number) + '.png')
