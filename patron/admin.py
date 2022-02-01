from django.contrib import admin


# Register your models here.
from patron.models import Image, LFont


class ImageAdmin(admin.ModelAdmin):
    fields = ['name', 'image', 'paper_size', 'num_img', 'labels', 'rend_size']
    list_display = ['id', 'name', 'image', 'paper_size', 'num_img', 'labels', 'rend_size']


class LFontAdmin(admin.ModelAdmin):
    fields = ['font']
    list_display = ['id', 'font']


admin.site.register(Image, ImageAdmin)
admin.site.register(LFont, LFontAdmin)