from django.contrib import admin


# Register your models here.
from patron.models import Image


class ImageAdmin(admin.ModelAdmin):
    fields = ['name', 'image', 'paper_size', 'num_img', 'labels']
    list_display = ['id', 'name', 'image', 'paper_size', 'num_img', 'labels']


admin.site.register(Image, ImageAdmin)
