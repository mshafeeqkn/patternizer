from django.contrib import admin


# Register your models here.
from patron.models import Image


class ImageAdmin(admin.ModelAdmin):
    fields = ['name', 'image']
    list_display = ['id', 'name', 'image']


admin.site.register(Image, ImageAdmin)
